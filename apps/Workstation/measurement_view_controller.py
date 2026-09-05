"""Workstation measurement and alignment presentation controller.

Inputs retain their producer frames: PX4 attitude body-FRD to earth-NED,
GPS/RTK baseline NED, OpenCV optical RDF, and public measurement
baseline-center FRD. Conversions remain explicit in workstation_core helpers.
"""

from __future__ import annotations

import hashlib
import html as html_lib
import json
import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, replace
from math import isfinite

from PyQt5 import QtCore

from Workstation.workstation_core.attitude_geometry import relative_attitude, world_ned_vector_to_body_frd
from Workstation.workstation_core.calibration import (
    StereoExtrinsics,
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
    compose_stereo_calibration,
    intrinsics_content_sha256,
    without_stereo_extrinsics,
)
from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    COMBINED_HYBRID,
    DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
    DRONE_RTK_MULTIVIEW,
    DRONE_RTK_ROTATION_HYBRID,
    HORIZON_HYBRID,
    MEASUREMENT_FILL_LIVE_GNSS,
    MEASUREMENT_FILL_LIVE_PIXHAWK,
    MEASUREMENT_FILL_NONE,
    MEASUREMENT_METHOD_IDS,
    MEASUREMENT_METHOD_LABELS,
    MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS,
    MEASUREMENT_METHOD_REQUIRED_INPUTS,
    MEASUREMENT_POSE_DOFS,
    MEASUREMENT_POSE_SIDES,
    MOUNTAIN_GNSS,
    PIXHAWK_RTK_LIVE,
    PIXHAWK_RP_GPS_BASELINE_YAW,
    TERRAIN_PNP,
    MeasurementMethodStatus,
    choose_ready_measurement_method,
    compatible_measurement_fill_source,
    default_measurement_method_fill_policy,
    latest_status_by_method,
    measurement_method_id_for_candidate,
    normalize_measurement_method_fallback_order,
    normalize_measurement_method_fill_policy,
    normalize_measurement_method_id,
)
from Workstation.workstation_core.optical_pose import (
    POSE_DOF_NAMES,
    PoseParametersNed,
    estimate_drone_yaw_deg,
    pose_parameters_from_camera_pose,
    stereo_pose_from_absolute,
)
from Workstation.workstation_core.operating_intrinsics import (
    NOMINAL_INTRINSICS_QUALITY,
    nominal_profile_to_stereo_result,
)
from Workstation.workstation_core.gnss_operation import (
    GnssOperationState,
    GnssReadinessSnapshot,
    ReadinessConfig,
)
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    BlockedMeasurementPair,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
    build_checkerboard_baseline_snapshot,
    build_pixhawk_baseline_snapshot,
    detection_pair_key,
    live_pair_timing_evidence,
)
from Workstation.workstation_core.baseline_yaw import (
    BaselineYawSelection,
    quaternion_from_roll_pitch_yaw_deg,
    select_baseline_yaw,
)
from Workstation.workstation_core.measurement_controller import calculate_measurement_geometry
from Workstation.workstation_core.measurement_data_store import (
    RETRY_RESERVE_MAX_BYTES,
    ROW_WRITE_ADMITTED,
    ROW_WRITE_UNAVAILABLE,
)
from Workstation.workstation_core.method_evaluation import (
    FrozenMeasurementPairSnapshot,
    MethodEvaluationBundle,
    MethodEvaluationOutcome,
    MethodEvaluationRequest,
)
from Workstation.workstation_core.method_evaluation_workers import (
    MethodEvaluationCoordinator,
)
from Workstation.workstation_core.detection_pair_coordinator import (
    DetectionPairCoordinator,
    PairTimingDecision,
    PairingPolicy,
)
from Workstation.workstation_core.native_evidence_writer import (
    RingPairReservation,
    reserve_ring_pair,
)
from Workstation.workstation_core.native_frame_ring import FrameKey
from Workstation.workstation_core.preview_measurement_worker import (
    PreviewMeasurementCoordinator,
    PreviewMeasurementResult,
)
from Workstation.workstation_core.settings import normalize_live_distance_time_basis
from Workstation.workstation_core.measurement_widgets import triangulation_3d_scene_points
from Workstation.workstation_core.rtk_geometry import (
    GpsPairReadiness,
    GpsPositionState,
    RtkBaselineEstimate,
    estimate_rtk_baseline,
    ned_delta_m,
    validate_gps_pair_for_baseline,
)
from Workstation.workstation_core.stereo import (
    DetectionObservation,
    StereoEstimate,
    TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC,
    detection_timestamp_for_basis,
)
from Workstation.workstation_core.sync import classify_attitude_timing
from Workstation.workstation_core.theme import (
    ERROR, ERROR_BG, ERROR_TEXT, PANEL_ALT_BG, PANEL_BORDER,
    SUCCESS, SUCCESS_BG, SUCCESS_TEXT, TEXT_MUTED,
    WARNING, WARNING_BG, WARNING_TEXT,
)
from Workstation.workstation_core.triangulation import (
    RayTriangulationResult,
    body_frd_vector_to_camera,
    camera_translation_right_in_left_camera,
    invert_left_to_right_camera_transform,
    triangulate_pixels_with_pixhawk_mount_attempt,
    triangulate_pixels_with_relative_pose_attempt,
)

GEOMETRY_COMPARISON_ANALYSIS_SCHEMA = "workstation-geometry-comparison-analysis-v1"
GEOMETRY_COMPARISON_ANALYSIS_NAMESPACE = "analysis/geometry_comparison"
DETECTION_PAIR_MAX_OBSERVATION_AGE_S = 1.0
LIVE_DISTANCE_MAX_PAIR_DELTA_S = 0.100
PRIMARY_MEASUREMENT_RETRY_BUDGET = 1
METHOD_BUNDLE_RETRY_BUDGET = 1
PERSISTENCE_PENDING_MAX_COUNT = 1024
PERSISTENCE_PENDING_MAX_BYTES = RETRY_RESERVE_MAX_BYTES
METHOD_EVALUATION_SHUTDOWN_CLOSE_RETRY_LIMIT = 8


@dataclass
class _PendingMethodBundleCommit:
    bundle: MethodEvaluationBundle
    row_owner: object | None = None
    row_index: int | None = None
    row_lease: tuple[int, int] | None = None

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
    "_distance_style"
]


class MeasurementViewController:
    def __init__(self, panel):
        self.panel = panel
        self._method_evaluation_coordinator: MethodEvaluationCoordinator | None = None
        self._method_evaluation_timer: QtCore.QTimer | None = None
        self._retiring_method_evaluation_coordinators: list[
            MethodEvaluationCoordinator
        ] = []
        self._detection_pair_coordinator: DetectionPairCoordinator | None = None
        self._detection_pair_timer: QtCore.QTimer | None = None
        self._pairing_policy_token: tuple[object, ...] | None = None
        self._pairing_policy_revision = 0
        self._preview_measurement_coordinator: PreviewMeasurementCoordinator | None = None
        self._preview_measurement_timer: QtCore.QTimer | None = None
        self._preview_measurement_results: dict[str, PreviewMeasurementResult] = {}
        self._preview_measurement_pending: set[str] = set()
        self._primary_outcome_hash_by_key: dict[tuple[str, str, int], str] = {}
        self._primary_normalized_outcome_by_key: dict[
            tuple[str, str, int], MethodEvaluationOutcome
        ] = {}
        self._primary_measurement_persisted: set[tuple[str, str, int]] = set()
        self._pending_primary_measurement_commits: dict[
            tuple[str, str, int], MethodEvaluationOutcome
        ] = {}
        self._pending_primary_measurement_sizes: dict[
            tuple[str, str, int], int
        ] = {}
        self._pending_primary_measurement_bytes = 0
        self._primary_history_projected: set[tuple[str, str, int]] = set()
        self._primary_display_high_water = -1
        self._primary_displayed_key: tuple[str, str, int] | None = None
        self._pending_method_bundle_commits: dict[
            tuple[str, str, int], _PendingMethodBundleCommit
        ] = {}
        self._pending_method_bundle_sizes: dict[tuple[str, str, int], int] = {}
        self._pending_method_bundle_bytes = 0
        self._persisted_method_bundle_keys: set[tuple[str, str, int]] = set()
        self._persistence_pending_overflow_count = 0
        self._persistence_pending_overflow_evidence_failures = 0
        self._last_persistence_pending_overflow: dict[str, object] = {}
        self._method_evaluation_shutdown_close_retry_calls = 0
        self._sync_persistence_pending_status()

    def _ensure_method_evaluation_timer(self) -> None:
        if (
            self._method_evaluation_timer is not None
            or QtCore.QCoreApplication.instance() is None
        ):
            return
        parent = self.panel if isinstance(self.panel, QtCore.QObject) else None
        timer = QtCore.QTimer(parent)
        timer.setInterval(20)
        timer.timeout.connect(self._poll_method_evaluations)
        timer.start()
        self._method_evaluation_timer = timer

    def _stop_method_evaluation_timer(self) -> None:
        timer = self._method_evaluation_timer
        if timer is None:
            return
        timer.stop()
        timer.deleteLater()
        self._method_evaluation_timer = None

    @staticmethod
    def _pending_record_size(record: Mapping[str, object]) -> int:
        try:
            return len(
                json.dumps(
                    dict(record),
                    ensure_ascii=False,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            )
        except (TypeError, ValueError):
            return len(repr(dict(record)).encode("utf-8", errors="replace"))

    def _sync_persistence_pending_status(self) -> None:
        self.panel._primary_measurement_retry_status = {
            "state": (
                "OVERFLOW"
                if self._persistence_pending_overflow_count
                else "PENDING"
                if (
                    self._pending_primary_measurement_commits
                    or self._pending_method_bundle_commits
                )
                else "IDLE"
            ),
            "primary_pending_count": len(
                self._pending_primary_measurement_commits
            ),
            "primary_pending_bytes": self._pending_primary_measurement_bytes,
            "bundle_pending_count": len(self._pending_method_bundle_commits),
            "bundle_pending_bytes": self._pending_method_bundle_bytes,
            "max_count_per_kind": max(1, int(PERSISTENCE_PENDING_MAX_COUNT)),
            "max_bytes_per_kind": max(1, int(PERSISTENCE_PENDING_MAX_BYTES)),
            "overflow_count": self._persistence_pending_overflow_count,
            "overflow_evidence_failures": (
                self._persistence_pending_overflow_evidence_failures
            ),
            "shutdown_close_retry_calls": (
                self._method_evaluation_shutdown_close_retry_calls
            ),
            "shutdown_close_retry_limit": max(
                1,
                int(METHOD_EVALUATION_SHUTDOWN_CLOSE_RETRY_LIMIT),
            ),
            "last_overflow": dict(self._last_persistence_pending_overflow),
        }

    def _record_persistence_pending_overflow(
        self,
        *,
        kind: str,
        key: tuple[str, str, int],
        digest: str,
        reason: str,
    ) -> None:
        self._persistence_pending_overflow_count += 1
        record = {
            "event_type": "measurement_persistence_retry_overflow",
            "pending_kind": str(kind),
            "eviction_policy": "oldest_first_keep_newest",
            "reason": str(reason),
            "measurement_id": str(key[0]),
            "pair_id": str(key[1]),
            "request_generation": int(key[2]),
            "evidence_sha256": str(digest),
            "primary_pending_count": len(
                self._pending_primary_measurement_commits
            ),
            "primary_pending_bytes": self._pending_primary_measurement_bytes,
            "bundle_pending_count": len(self._pending_method_bundle_commits),
            "bundle_pending_bytes": self._pending_method_bundle_bytes,
            "max_count_per_kind": max(1, int(PERSISTENCE_PENDING_MAX_COUNT)),
            "max_bytes_per_kind": max(1, int(PERSISTENCE_PENDING_MAX_BYTES)),
        }
        writer = getattr(self.panel, "append_workstation_data", None)
        try:
            evidence_admitted = bool(
                callable(writer) and writer("events", record, priority=True)
            )
        except Exception:  # noqa: BLE001 - overflow accounting must always survive.
            evidence_admitted = False
        if not evidence_admitted:
            self._persistence_pending_overflow_evidence_failures += 1
        record["overflow_evidence_admitted"] = evidence_admitted
        self._last_persistence_pending_overflow = record
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            logger(
                "[측정] persistence retry overflow: "
                f"kind={kind}, pair={key[1]}, reason={reason}, "
                f"evidence={'QUEUED' if evidence_admitted else 'REJECTED'}"
            )
        self._sync_persistence_pending_status()

    def _release_recording_primary_retry(
        self,
        outcome: MethodEvaluationOutcome,
    ) -> None:
        recording_owner = getattr(
            self.panel,
            "_recording_controller_owner",
            None,
        )
        releaser = getattr(
            recording_owner,
            "_release_primary_retry_ownership",
            None,
        )
        if not callable(releaser):
            return
        try:
            attempt = MeasurementAttempt.from_canonical_payload(
                outcome.attempt_payload
            )
            releaser(tuple(attempt.detection_pair_key))
        except (KeyError, RuntimeError, TypeError, ValueError):
            return

    def _remove_pending_primary_measurement(
        self,
        key: tuple[str, str, int],
    ) -> None:
        self._pending_primary_measurement_commits.pop(key, None)
        self._pending_primary_measurement_bytes -= self._pending_primary_measurement_sizes.pop(
            key,
            0,
        )
        self._pending_primary_measurement_bytes = max(
            0,
            self._pending_primary_measurement_bytes,
        )
        self._sync_persistence_pending_status()

    def _retain_pending_primary_measurement(
        self,
        key: tuple[str, str, int],
        outcome: MethodEvaluationOutcome,
    ) -> bool:
        self._remove_pending_primary_measurement(key)
        size = self._pending_record_size(outcome.to_record())
        count_limit = max(1, int(PERSISTENCE_PENDING_MAX_COUNT))
        byte_limit = max(1, int(PERSISTENCE_PENDING_MAX_BYTES))
        if size > byte_limit:
            self._release_recording_primary_retry(outcome)
            self._record_persistence_pending_overflow(
                kind="primary_measurement",
                key=key,
                digest=outcome.canonical_sha256(),
                reason=f"single_record_bytes={size} exceeds byte_limit={byte_limit}",
            )
            return False
        while (
            len(self._pending_primary_measurement_commits) >= count_limit
            or self._pending_primary_measurement_bytes + size > byte_limit
        ):
            oldest_key = next(iter(self._pending_primary_measurement_commits))
            oldest = self._pending_primary_measurement_commits.get(oldest_key)
            oldest_size = self._pending_primary_measurement_sizes.pop(oldest_key, 0)
            self._pending_primary_measurement_commits.pop(oldest_key, None)
            self._pending_primary_measurement_bytes = max(
                0,
                self._pending_primary_measurement_bytes - oldest_size,
            )
            if oldest is not None:
                self._release_recording_primary_retry(oldest)
                self._record_persistence_pending_overflow(
                    kind="primary_measurement",
                    key=oldest_key,
                    digest=oldest.canonical_sha256(),
                    reason="oldest evicted to admit newer primary retry",
                )
        self._pending_primary_measurement_commits[key] = outcome
        self._pending_primary_measurement_sizes[key] = size
        self._pending_primary_measurement_bytes += size
        self._sync_persistence_pending_status()
        return True

    def _remove_pending_method_bundle(
        self,
        key: tuple[str, str, int],
    ) -> None:
        self._pending_method_bundle_commits.pop(key, None)
        self._pending_method_bundle_bytes -= self._pending_method_bundle_sizes.pop(
            key,
            0,
        )
        self._pending_method_bundle_bytes = max(
            0,
            self._pending_method_bundle_bytes,
        )
        self._sync_persistence_pending_status()

    def _retain_pending_method_bundle(
        self,
        key: tuple[str, str, int],
        pending: _PendingMethodBundleCommit,
    ) -> bool:
        self._remove_pending_method_bundle(key)
        size = self._pending_record_size(pending.bundle.to_record())
        count_limit = max(1, int(PERSISTENCE_PENDING_MAX_COUNT))
        byte_limit = max(1, int(PERSISTENCE_PENDING_MAX_BYTES))
        if size > byte_limit:
            self._record_persistence_pending_overflow(
                kind="method_bundle",
                key=key,
                digest=pending.bundle.canonical_sha256(),
                reason=f"single_record_bytes={size} exceeds byte_limit={byte_limit}",
            )
            return False
        while (
            len(self._pending_method_bundle_commits) >= count_limit
            or self._pending_method_bundle_bytes + size > byte_limit
        ):
            oldest_key = next(iter(self._pending_method_bundle_commits))
            oldest = self._pending_method_bundle_commits.get(oldest_key)
            oldest_size = self._pending_method_bundle_sizes.pop(oldest_key, 0)
            self._pending_method_bundle_commits.pop(oldest_key, None)
            self._pending_method_bundle_bytes = max(
                0,
                self._pending_method_bundle_bytes - oldest_size,
            )
            if oldest is not None:
                self._record_persistence_pending_overflow(
                    kind="method_bundle",
                    key=oldest_key,
                    digest=oldest.bundle.canonical_sha256(),
                    reason="oldest evicted to admit newer bundle retry",
                )
        self._pending_method_bundle_commits[key] = pending
        self._pending_method_bundle_sizes[key] = size
        self._pending_method_bundle_bytes += size
        self._sync_persistence_pending_status()
        return True

    def _terminalize_shutdown_pending_retries(self) -> None:
        attempts = self._method_evaluation_shutdown_close_retry_calls
        reason = (
            "application shutdown close retry budget exhausted: "
            f"attempts={attempts}"
        )
        for key, outcome in tuple(
            self._pending_primary_measurement_commits.items()
        ):
            self._release_recording_primary_retry(outcome)
            self._record_persistence_pending_overflow(
                kind="primary_measurement_shutdown_drop",
                key=key,
                digest=outcome.canonical_sha256(),
                reason=reason,
            )
            self._remove_pending_primary_measurement(key)
        for key, pending in tuple(self._pending_method_bundle_commits.items()):
            self._record_persistence_pending_overflow(
                kind="method_bundle_shutdown_drop",
                key=key,
                digest=pending.bundle.canonical_sha256(),
                reason=reason,
            )
            self._remove_pending_method_bundle(key)
        self._sync_persistence_pending_status()

    def _ensure_method_evaluation_coordinator(self) -> MethodEvaluationCoordinator:
        coordinator = self._method_evaluation_coordinator
        if coordinator is None:
            coordinator = MethodEvaluationCoordinator()
            self._method_evaluation_coordinator = coordinator
        self._ensure_method_evaluation_timer()
        return coordinator

    def close_method_evaluation_workers(self) -> bool:
        self._retry_pending_primary_measurements()
        self._retry_pending_method_bundles()
        coordinator = self._method_evaluation_coordinator
        self._method_evaluation_coordinator = None
        if coordinator is not None:
            coordinator.close()
            self._retiring_method_evaluation_coordinators.append(coordinator)
        remaining: list[MethodEvaluationCoordinator] = []
        for retiring in self._retiring_method_evaluation_coordinators:
            retiring.close()
            if not retiring.closed:
                remaining.append(retiring)
        self._retiring_method_evaluation_coordinators = remaining
        pending_persistence = bool(
            self._pending_primary_measurement_commits
            or self._pending_method_bundle_commits
        )
        if bool(getattr(self.panel, "_shutdown_in_progress", False)):
            if pending_persistence:
                self._method_evaluation_shutdown_close_retry_calls += 1
                if self._method_evaluation_shutdown_close_retry_calls >= max(
                    1,
                    int(METHOD_EVALUATION_SHUTDOWN_CLOSE_RETRY_LIMIT),
                ):
                    self._terminalize_shutdown_pending_retries()
            else:
                self._method_evaluation_shutdown_close_retry_calls = 0
        else:
            # Measurement generation retirement ignores this return value and
            # leaves retry ownership alive; it must never consume shutdown budget.
            self._method_evaluation_shutdown_close_retry_calls = 0
        self._sync_persistence_pending_status()
        if (
            self._pending_primary_measurement_commits
            or self._pending_method_bundle_commits
        ):
            self._ensure_method_evaluation_timer()
        else:
            self._stop_method_evaluation_timer()
        detection_timer = self._detection_pair_timer
        if detection_timer is not None:
            detection_timer.stop()
            detection_timer.deleteLater()
            self._detection_pair_timer = None
        detection_coordinator = self._detection_pair_coordinator
        self._detection_pair_coordinator = None
        detection_closed = (
            True
            if detection_coordinator is None
            else detection_coordinator.close()
        )
        self._release_detection_pair_reservation(
            getattr(self.panel, "_coordinated_detection_pair_reservation", None)
        )
        self.panel._coordinated_detection_pair_reservation = None
        preview_timer = self._preview_measurement_timer
        if preview_timer is not None:
            preview_timer.stop()
            preview_timer.deleteLater()
            self._preview_measurement_timer = None
        preview_coordinator = self._preview_measurement_coordinator
        self._preview_measurement_coordinator = None
        preview_closed = (
            True
            if preview_coordinator is None
            else preview_coordinator.close()
        )
        self._preview_measurement_results.clear()
        self._preview_measurement_pending.clear()
        return bool(
            not remaining
            and detection_closed
            and preview_closed
            and not self._pending_primary_measurement_commits
            and not self._pending_method_bundle_commits
        )

    def _measurement_method_state(self) -> None:
        panel = self.panel
        measurement_id = str(getattr(panel, "_measurement_id", "") or "")
        package = getattr(panel, "_measurement_package", None)
        run_metadata = getattr(panel, "_measurement_run_metadata", None)
        config_owner = package if package is not None else run_metadata
        config_token = (measurement_id, id(config_owner) if config_owner is not None else 0)
        if getattr(panel, "_measurement_method_config_token", None) != config_token:
            metadata: dict[str, object] = (
                dict(run_metadata) if isinstance(run_metadata, Mapping) else {}
            )
            snapshotter = getattr(package, "snapshot", None)
            if measurement_id and callable(snapshotter):
                try:
                    package_snapshot = snapshotter()
                    if isinstance(package_snapshot, Mapping):
                        raw_metadata = package_snapshot.get("metadata")
                        if isinstance(raw_metadata, Mapping):
                            metadata = dict(raw_metadata)
                except Exception:
                    metadata = {}
            settings = getattr(panel, "settings", None)
            setting_reader = getattr(panel, "_workstation_setting", None)
            if not callable(setting_reader):
                setting_reader = lambda _name, default=None: default
            global_preferred = normalize_measurement_method_id(
                setting_reader(
                    "measurement_preferred_method",
                    getattr(settings, "measurement_preferred_method", PIXHAWK_RTK_LIVE),
                )
            )
            global_order = normalize_measurement_method_fallback_order(
                setting_reader(
                    "measurement_method_fallback_order",
                    getattr(
                        settings,
                        "measurement_method_fallback_order",
                        DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
                    ),
                )
            )
            global_reverse = bool(
                setting_reader(
                    "measurement_baseline_yaw_reverse",
                    getattr(settings, "measurement_baseline_yaw_reverse", False),
                )
            )
            panel._measurement_preferred_method_id = normalize_measurement_method_id(
                metadata.get("measurement_preferred_method", global_preferred)
            )
            panel._measurement_active_fallback_order = normalize_measurement_method_fallback_order(
                metadata.get("measurement_method_fallback_order", global_order)
            )
            panel._measurement_baseline_yaw_reverse = bool(
                metadata.get("measurement_baseline_yaw_reverse", global_reverse)
            )
            global_time_basis = self._configured_time_basis()
            panel._measurement_active_time_basis_requested = (
                normalize_live_distance_time_basis(
                    metadata.get(
                        "measurement_pairing_time_basis_requested",
                        global_time_basis,
                    )
                )
            )
            panel._measurement_method_fill_policy = normalize_measurement_method_fill_policy(
                metadata.get("measurement_method_fill_policy")
            )
            frozen_inputs = metadata.get("measurement_frozen_comparison_inputs")
            panel._measurement_frozen_comparison_inputs_declared = (
                "measurement_frozen_comparison_inputs" in metadata
            )
            panel._measurement_frozen_comparison_inputs = (
                dict(frozen_inputs) if isinstance(frozen_inputs, Mapping) else {}
            )
            panel._measurement_method_config_token = config_token
        if not hasattr(panel, "_measurement_selected_method_id"):
            panel._measurement_selected_method_id = ""
            panel._measurement_selected_instance_id = ""
            panel._measurement_selection_reason = "startup"
            panel._measurement_method_transition_sequence = 0
            panel._measurement_method_ready_since = {}

    def measurement_method_config_for_new_measurement(self) -> dict[str, object]:
        """Freeze global method defaults into a new Measurement boundary."""

        panel = self.panel
        preferred = normalize_measurement_method_id(
            panel._workstation_setting(
                "measurement_preferred_method",
                getattr(panel.settings, "measurement_preferred_method", PIXHAWK_RTK_LIVE),
            )
        )
        order = normalize_measurement_method_fallback_order(
            panel._workstation_setting(
                "measurement_method_fallback_order",
                getattr(
                    panel.settings,
                    "measurement_method_fallback_order",
                    DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
                ),
            )
        )
        owner = panel._optical_pose_controller()
        try:
            intrinsics_snapshot = owner._intrinsics_snapshot(use_frozen=False)
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError):
            intrinsics_snapshot = {}
        intrinsics_snapshot = (
            dict(intrinsics_snapshot)
            if isinstance(intrinsics_snapshot, Mapping)
            else {}
        )
        applicability = intrinsics_snapshot.get("applicability")
        applicability = applicability if isinstance(applicability, Mapping) else {}
        operating_intrinsics = {
            "profile_sha256": str(applicability.get("profile_sha256") or ""),
            "geometry_allowed": bool(applicability.get("geometry_allowed", False)),
            "reasons": list(applicability.get("reasons") or ()),
            "profile": (
                dict(intrinsics_snapshot.get("profile"))
                if isinstance(intrinsics_snapshot.get("profile"), Mapping)
                else None
            ),
        }
        frozen_intrinsics_result = getattr(panel, "active_intrinsics", None)
        operating_profile_reader = getattr(
            owner, "operating_intrinsics_profile_for_geometry", None
        )
        try:
            operating_profile = (
                operating_profile_reader()
                if callable(operating_profile_reader)
                else owner._operating_intrinsics_profile()
            )
        except (AttributeError, RuntimeError, TypeError, ValueError):
            operating_profile = None
        if (
            operating_profile is not None
            and operating_profile.provenance.get("quality")
            == NOMINAL_INTRINSICS_QUALITY
        ):
            frozen_intrinsics_result = nominal_profile_to_stereo_result(
                operating_profile
            )
        panel._pending_measurement_frozen_intrinsics_result = (
            None
            if frozen_intrinsics_result is None
            else without_stereo_extrinsics(frozen_intrinsics_result)
        )
        panel._pending_measurement_frozen_intrinsics_path = str(
            getattr(operating_profile, "source_path", "")
            or getattr(panel, "_active_intrinsics_profile_path", "")
            or ""
        )
        latest_candidates: dict[str, object] = {}
        for candidate in getattr(owner, "_candidates", {}).values():
            method_id = measurement_method_id_for_candidate(candidate.method)
            if method_id is None:
                continue
            current = latest_candidates.get(method_id)
            if current is None or int(candidate.created_utc_ns or 0) >= int(
                getattr(current, "created_utc_ns", 0) or 0
            ):
                latest_candidates[method_id] = candidate
        frozen_inputs: dict[str, object] = {
            PIXHAWK_RTK_LIVE: {
                "kind": "live_at_pair",
                "instance_id": "live:pixhawk-rtk",
                "operating_intrinsics": operating_intrinsics,
                "live_inputs": ["left/right Pixhawk attitude", "P1/P2 GNSS"],
            },
            PIXHAWK_RP_GPS_BASELINE_YAW: {
                "kind": "live_at_pair",
                "instance_id": "live:pixhawk-rp-gps-baseline-yaw",
                "operating_intrinsics": operating_intrinsics,
                "live_inputs": [
                    "left/right Pixhawk Roll/Pitch",
                    "P1/P2 GNSS baseline",
                    "P1/P2 Pixhawk yaw branch reference",
                ],
                "yaw_contract": "P1→P2 NED azimuth +/- 90deg; persistent Reverse may invert",
            },
        }
        frozen_objects: dict[str, object] = {}
        checkerboard_draft = getattr(
            panel, "checkerboard_extrinsics_candidate_profile", None
        )
        if checkerboard_draft is not None:
            frozen_inputs[CHECKERBOARD_STEREO] = {
                "kind": "checkerboard_draft",
                "instance_id": (
                    f"frozen-draft:{checkerboard_draft.profile_id or checkerboard_draft.content_sha256}"
                ),
                "profile_id": checkerboard_draft.profile_id,
                "content_sha256": checkerboard_draft.content_sha256,
                "profile": asdict(checkerboard_draft),
                "operating_intrinsics": operating_intrinsics,
            }
            frozen_objects[CHECKERBOARD_STEREO] = checkerboard_draft
        for method_id, candidate in latest_candidates.items():
            prior = getattr(owner, "_candidate_priors", {}).get(candidate.candidate_id)
            mapper = getattr(owner, "_candidate_mapping_with_prior", None)
            mapping = (
                mapper(candidate, prior)
                if callable(mapper) and isinstance(prior, tuple) and len(prior) == 2
                else {
                    "candidate_id": candidate.candidate_id,
                    "method": candidate.method,
                    "created_utc_ns": candidate.created_utc_ns,
                }
            )
            frozen_inputs[method_id] = {
                "kind": "optical_pose_draft",
                "instance_id": f"frozen-draft:{candidate.candidate_id}",
                "candidate_id": candidate.candidate_id,
                "candidate": mapping,
                "operating_intrinsics": operating_intrinsics,
            }
            frozen_objects[method_id] = candidate
        for method_id in MEASUREMENT_METHOD_IDS[1:]:
            frozen_inputs.setdefault(
                method_id,
                {
                    "kind": "missing_draft",
                    "instance_id": f"{method_id}:frozen-input-missing",
                    "draft_missing": True,
                    "operating_intrinsics": operating_intrinsics,
                },
            )
        frozen_inputs = json.loads(
            json.dumps(
                frozen_inputs,
                ensure_ascii=False,
                allow_nan=False,
                default=str,
            )
        )
        panel._pending_measurement_frozen_comparison_objects = frozen_objects
        panel._measurement_frozen_comparison_inputs = frozen_inputs
        return {
            "measurement_method_config_schema": "measurement-method-config-v1",
            "measurement_preferred_method": preferred,
            "measurement_method_fallback_order": list(order),
            "measurement_method_fill_policy": default_measurement_method_fill_policy(),
            "measurement_baseline_yaw_reverse": bool(
                panel._workstation_setting(
                    "measurement_baseline_yaw_reverse",
                    getattr(panel.settings, "measurement_baseline_yaw_reverse", False),
                )
            ),
            "measurement_frozen_comparison_inputs": frozen_inputs,
            "measurement_pairing_time_basis_requested": self._configured_time_basis(),
        }

    def _measurement_fill_policy(self) -> dict[str, dict[str, str]]:
        self._measurement_method_state()
        return normalize_measurement_method_fill_policy(
            getattr(self.panel, "_measurement_method_fill_policy", None)
        )

    def _measurement_fill_source(self, method_id: str, side: str, dof: str) -> str:
        return self._measurement_fill_policy()[method_id][f"{side}.{dof}"]

    def _measurement_live_readiness(
        self,
    ) -> tuple[bool, bool, tuple[str, ...], tuple[str, ...]]:
        """Return field-policy hard readiness plus telemetry advisories."""

        now_s = time.monotonic()
        defaults = ReadinessConfig()
        hard_reasons: list[str] = []
        advisories: list[str] = []
        left_attitude, right_attitude, reused_attitude = (
            self._cached_measurement_attitude_states()
        )
        for side, sample in (("left", left_attitude), ("right", right_attitude)):
            if sample is None:
                hard_reasons.append(f"{side} Pixhawk attitude missing")
                continue
            timestamp_s = getattr(sample, "timestamp_s", None)
            try:
                age_s = now_s - float(timestamp_s)
            except (TypeError, ValueError):
                age_s = float("nan")
            if not math.isfinite(age_s) or age_s < 0.0:
                hard_reasons.append(f"{side} Pixhawk attitude timestamp invalid")
            elif age_s > 2.0:
                advisories.append(f"{side} Pixhawk attitude stale ({age_s:.3f}s)")
        if reused_attitude:
            advisories.append(
                "Pixhawk attitude reused from latest valid sample: "
                + ",".join(reused_attitude)
            )
        pixhawk_hard = tuple(reason for reason in hard_reasons if "Pixhawk" in reason)
        pixhawk_ready = not pixhawk_hard
        gnss_ready = False
        left_gps, right_gps, reused_gps = self._cached_measurement_gps_states()
        dropped_frame_sides = tuple(
            str(side)
            for side in (
                getattr(self.panel, "_measurement_gps_correction_frame_dropped", ())
                or ()
            )
        )
        if dropped_frame_sides:
            advisories.append(
                "GNSS correction frame changed; retained fix discarded for "
                + ",".join(dropped_frame_sides)
                + "; reconnect that board to refix under the new reference"
            )
        if left_gps is None or right_gps is None:
            hard_reasons.append("live GNSS baseline missing")
        elif not left_gps.has_position or not right_gps.has_position or not all(
            math.isfinite(float(value))
            for sample in (left_gps, right_gps)
            for value in (sample.latitude_deg, sample.longitude_deg, sample.altitude_m)
        ):
            hard_reasons.append("GNSS position missing or non-finite")
        else:
            ned = ned_delta_m(left_gps, right_gps)
            baseline_m = (
                float("nan")
                if ned is None
                else math.sqrt(sum(float(value) ** 2 for value in ned))
            )
            if not math.isfinite(baseline_m) or baseline_m <= 0.0:
                hard_reasons.append("GPS baseline length <= 0 or non-finite")
            else:
                gnss_ready = True
                baseline = getattr(self.panel, "latest_rtk_baseline", None)
                advisories.extend(tuple(getattr(baseline, "warnings", ()) or ()))
                usability = getattr(
                    self.panel, "_gps_baseline_usable_for_translation", None
                )
                if baseline is not None and callable(usability):
                    _usable, reason = usability(baseline)
                    if reason and not _usable:
                        advisories.append(str(reason))
                for side, sample in (("left", left_gps), ("right", right_gps)):
                    if int(getattr(sample, "fix_type", 0) or 0) != 6:
                        advisories.append(
                            f"{side} {getattr(sample, 'rtk_label', 'non-RTK-fixed')}"
                        )
                if reused_gps:
                    advisories.append(
                        "GNSS reused from latest valid 3D fix: " + ",".join(reused_gps)
                    )
                # P1 and P2 may only be connectable one at a time, so the
                # retained pair can span minutes.  The exact span is reported
                # instead of blocked: sequential capture is the only available
                # field procedure on such a rig.
                try:
                    pair_delta_s = abs(
                        float(left_gps.timestamp_s) - float(right_gps.timestamp_s)
                    )
                except (AttributeError, TypeError, ValueError):
                    pair_delta_s = float("nan")
                maximum_pair_delta_s = (
                    float(
                        getattr(
                            self._gnss_readiness_config(),
                            "maximum_left_right_timestamp_delta_ms",
                            defaults.maximum_left_right_timestamp_delta_ms,
                        )
                    )
                    * 0.001
                )
                if math.isfinite(pair_delta_s) and pair_delta_s > maximum_pair_delta_s:
                    advisories.append(
                        f"P1/P2 GPS pair captured {pair_delta_s:.3f}s apart "
                        f"(limit {maximum_pair_delta_s:.3f}s)"
                    )
        if gnss_ready and bool(getattr(self.panel, "gnss_operation_gate_active", False)):
            manager = getattr(self.panel, "gnss_operation_manager", None)
            readiness = None if manager is None else manager.readiness()
            config = None if manager is None else manager.config
            if (
                config is not None
                and bool(getattr(config, "readiness_gate_enabled", False))
                and (readiness is None or not readiness.ready)
            ):
                gnss_warning = (
                    "GNSS readiness snapshot missing"
                    if readiness is None
                    else "; ".join(readiness.reasons) or "GNSS readiness blocked"
                )
                advisories.append(gnss_warning)
        return (
            pixhawk_ready,
            gnss_ready,
            tuple(dict.fromkeys(hard_reasons)),
            tuple(dict.fromkeys(advisories)),
        )

    @staticmethod
    def _candidate_observed_dofs(
        candidate: object,
        side: str | None = None,
    ) -> tuple[str, ...]:
        mask = getattr(candidate, "optimized_dofs", None)
        side_rows = dict(getattr(candidate, "observability_by_side", ()) or ())
        observable = dict(
            side_rows.get(side, getattr(candidate, "observability", ()) or ())
        )
        return tuple(
            name
            for name in POSE_DOF_NAMES
            if bool(getattr(mask, name, False)) and bool(observable.get(name, False))
        )

    @staticmethod
    def _geometry_evidence_sha256(**parts: object) -> str:
        return hashlib.sha256(
            json.dumps(
                parts,
                sort_keys=True,
                separators=(",", ":"),
            ).encode()
        ).hexdigest()

    def _operating_intrinsics_readiness(
        self,
        owner: object,
    ) -> tuple[bool, str, tuple[str, ...]]:
        """Return exact live K,D applicability and its immutable profile ID."""

        reasons: list[str] = []
        applicability: object = None
        profile: object = None
        try:
            snapshot = owner._intrinsics_snapshot(use_frozen=False)
            applicability = snapshot["applicability"]
            profile = snapshot.get("profile")
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError):
            reasons.append("exact operating K,D applicability unavailable")
        profile_row = profile if isinstance(profile, Mapping) else {}
        provenance = profile_row.get("provenance")
        provenance = provenance if isinstance(provenance, Mapping) else {}
        nominal_active = provenance.get("quality") == NOMINAL_INTRINSICS_QUALITY
        if getattr(self.panel, "active_intrinsics", None) is None and not nominal_active:
            reasons.append("exact operating K,D missing")
        row = applicability if isinstance(applicability, Mapping) else {}
        geometry_allowed = bool(row.get("geometry_allowed"))
        if not geometry_allowed:
            reasons.append("exact operating K,D not applicable")
            reasons.extend(
                f"K,D applicability: {reason}"
                for reason in tuple(row.get("reasons") or ())
            )
        profile_hash = str(row.get("profile_sha256") or "")
        if not (
            len(profile_hash) == 64
            and all(character in "0123456789abcdef" for character in profile_hash)
        ):
            reasons.append("exact operating K,D profile SHA-256 missing or invalid")
            profile_hash = ""
        stable_reasons = tuple(dict.fromkeys(reasons))
        return not stable_reasons, profile_hash, stable_reasons

    def _measurement_baseline_yaw_reverse(self) -> bool:
        self._measurement_method_state()
        setting_reader = getattr(self.panel, "_workstation_setting", None)
        if not callable(setting_reader):
            setting_reader = lambda _name, default=None: default
        settings = getattr(self.panel, "settings", None)
        default = getattr(settings, "measurement_baseline_yaw_reverse", False)
        if not str(getattr(self.panel, "_measurement_id", "") or ""):
            return bool(
                setting_reader(
                    "measurement_baseline_yaw_reverse",
                    default,
                )
            )
        if hasattr(self.panel, "_measurement_baseline_yaw_reverse"):
            return bool(self.panel._measurement_baseline_yaw_reverse)
        return bool(
            setting_reader(
                "measurement_baseline_yaw_reverse",
                default,
            )
        )

    def _baseline_yaw_selection_preview(
        self,
        *,
        left_gps: object | None = None,
        right_gps: object | None = None,
        left_attitude: object | None = None,
        right_attitude: object | None = None,
    ) -> tuple[BaselineYawSelection | None, float | None, float | None]:
        if left_gps is None or right_gps is None:
            selector = getattr(self.panel, "_selected_baseline_gps_states", None)
            if callable(selector):
                selected_left, selected_right = selector()
            else:
                latest_gps = getattr(self.panel, "latest_gps", {}) or {}
                selected_left = latest_gps.get("left") if isinstance(latest_gps, Mapping) else None
                selected_right = latest_gps.get("right") if isinstance(latest_gps, Mapping) else None
            left_gps = left_gps or selected_left
            right_gps = right_gps or selected_right
        if left_gps is None or right_gps is None:
            return None, None, None
        baseline_ned = ned_delta_m(left_gps, right_gps)
        if baseline_ned is None:
            return None, None, None
        if left_attitude is None or right_attitude is None:
            left_attitude, right_attitude, _reused = self._cached_measurement_attitude_states()
        p1_yaw = getattr(left_attitude, "yaw_deg", None)
        p2_yaw = getattr(right_attitude, "yaw_deg", None)
        now_s = time.monotonic()
        ages = []
        for sample in (left_attitude, right_attitude):
            timestamp_s = getattr(sample, "timestamp_s", None)
            try:
                ages.append(max(0.0, now_s - float(timestamp_s)))
            except (TypeError, ValueError):
                ages.append(None)
        try:
            selection = select_baseline_yaw(
                north_m=float(baseline_ned[0]),
                east_m=float(baseline_ned[1]),
                p1_yaw_deg=p1_yaw,
                p2_yaw_deg=p2_yaw,
                reverse=self._measurement_baseline_yaw_reverse(),
            )
        except (TypeError, ValueError):
            return None, ages[0], ages[1]
        return selection, ages[0], ages[1]

    def _measurement_method_statuses(self) -> tuple[MeasurementMethodStatus, ...]:
        """Project every geometry source into independent method readiness."""

        self._measurement_method_state()
        owner = self.panel._optical_pose_controller()
        frozen_inputs = getattr(
            self.panel, "_measurement_frozen_comparison_inputs", {}
        )
        frozen_inputs = frozen_inputs if isinstance(frozen_inputs, Mapping) else {}
        frozen_objects = getattr(
            self.panel, "_pending_measurement_frozen_comparison_objects", {}
        )
        frozen_objects = frozen_objects if isinstance(frozen_objects, Mapping) else {}
        measurement_active = bool(
            str(getattr(self.panel, "_measurement_id", "") or "")
            and bool(
                getattr(
                    self.panel,
                    "_measurement_frozen_comparison_inputs_declared",
                    False,
                )
            )
        )
        _geometry_allowed, current_profile_hash, intrinsics_reasons = (
            self._operating_intrinsics_readiness(owner)
        )
        if measurement_active:
            primary_input = frozen_inputs.get(PIXHAWK_RTK_LIVE)
            primary_input = (
                primary_input if isinstance(primary_input, Mapping) else {}
            )
            frozen_kd = primary_input.get("operating_intrinsics")
            frozen_kd = frozen_kd if isinstance(frozen_kd, Mapping) else {}
            current_profile_hash = str(frozen_kd.get("profile_sha256") or "")
            frozen_reasons: list[str] = []
            if not bool(frozen_kd.get("geometry_allowed", False)):
                frozen_reasons.append("frozen exact operating K,D not applicable")
                frozen_reasons.extend(
                    f"K,D applicability: {reason}"
                    for reason in frozen_kd.get("reasons", ()) or ()
                )
            if not (
                len(current_profile_hash) == 64
                and all(
                    character in "0123456789abcdef"
                    for character in current_profile_hash
                )
            ):
                frozen_reasons.append(
                    "frozen exact operating K,D profile SHA-256 missing or invalid"
                )
                current_profile_hash = ""
            if getattr(
                self.panel, "_pending_measurement_frozen_intrinsics_result", None
            ) is None:
                frozen_reasons.append("frozen exact operating K,D values missing")
            intrinsics_reasons = tuple(dict.fromkeys(frozen_reasons))
        pixhawk_ready, gnss_ready, live_hard_reasons, live_advisories = (
            self._measurement_live_readiness()
        )
        statuses: list[MeasurementMethodStatus] = []

        pixhawk_reasons = list(intrinsics_reasons)
        if not pixhawk_ready:
            pixhawk_reasons.extend(reason for reason in live_hard_reasons if "Pixhawk" in reason)
        if not gnss_ready:
            pixhawk_reasons.extend(reason for reason in live_hard_reasons if "Pixhawk" not in reason)
        statuses.append(
            MeasurementMethodStatus(
                method_id=PIXHAWK_RTK_LIVE,
                instance_id="live:pixhawk-rtk",
                ready=not pixhawk_reasons,
                reasons=tuple(dict.fromkeys(pixhawk_reasons)),
                geometry_sha256=current_profile_hash,
                observed_dofs=POSE_DOF_NAMES,
                component_sources=tuple(
                    (
                        f"{side}.{name}",
                        "live_pixhawk" if name.endswith("_deg") else "live_gnss",
                    )
                    for side in ("left", "right")
                    for name in POSE_DOF_NAMES
                ),
                requires_live_pixhawk=True,
                requires_live_gnss=True,
                warnings=live_advisories,
            )
        )

        baseline_yaw_selection, _p1_yaw_age_s, _p2_yaw_age_s = (
            self._baseline_yaw_selection_preview()
        )
        baseline_yaw_reasons = list(intrinsics_reasons)
        if not pixhawk_ready:
            baseline_yaw_reasons.extend(
                reason for reason in live_hard_reasons if "Pixhawk" in reason
            )
        if not gnss_ready:
            baseline_yaw_reasons.extend(
                reason for reason in live_hard_reasons if "Pixhawk" not in reason
            )
        if baseline_yaw_selection is None or not baseline_yaw_selection.ready:
            baseline_yaw_reasons.append(
                "baseline yaw branch unavailable"
                if baseline_yaw_selection is None
                else baseline_yaw_selection.reason
            )
        baseline_yaw_reverse = self._measurement_baseline_yaw_reverse()
        baseline_yaw_sources = tuple(
            (f"{side}.roll_deg", "live_pixhawk")
            for side in ("left", "right")
        ) + tuple(
            (f"{side}.pitch_deg", "live_pixhawk")
            for side in ("left", "right")
        ) + tuple(
            (f"{side}.yaw_deg", "gnss_baseline_yaw_reverse" if baseline_yaw_reverse else "gnss_baseline_yaw_auto")
            for side in ("left", "right")
        ) + tuple(
            (f"{side}.{name}", "live_gnss")
            for side in ("left", "right")
            for name in ("north_m", "east_m", "down_m")
        )
        baseline_yaw_warnings = list(live_advisories)
        if baseline_yaw_selection is not None:
            if baseline_yaw_selection.consensus is False:
                baseline_yaw_warnings.append("P1/P2 yaw branch preference disagrees")
            if baseline_yaw_selection.low_margin:
                baseline_yaw_warnings.append("baseline yaw branch score margin is low")
        statuses.append(
            MeasurementMethodStatus(
                method_id=PIXHAWK_RP_GPS_BASELINE_YAW,
                instance_id="live:pixhawk-rp-gps-baseline-yaw",
                ready=not baseline_yaw_reasons,
                reasons=tuple(dict.fromkeys(baseline_yaw_reasons)),
                geometry_sha256=self._geometry_evidence_sha256(
                    operating_intrinsics_profile_sha256=current_profile_hash,
                    method_id=PIXHAWK_RP_GPS_BASELINE_YAW,
                    reverse=baseline_yaw_reverse,
                    baseline_azimuth=(
                        None
                        if baseline_yaw_selection is None
                        else baseline_yaw_selection.baseline_azimuth_deg
                    ),
                ),
                observed_dofs=POSE_DOF_NAMES,
                component_sources=baseline_yaw_sources,
                requires_live_pixhawk=True,
                requires_live_gnss=True,
                warnings=tuple(dict.fromkeys(baseline_yaw_warnings)),
            )
        )

        checkerboard_row = frozen_inputs.get(CHECKERBOARD_STEREO)
        checkerboard_row = checkerboard_row if isinstance(checkerboard_row, Mapping) else {}
        checkerboard_profile = (
            frozen_objects.get(CHECKERBOARD_STEREO)
            if measurement_active
            else getattr(self.panel, "checkerboard_extrinsics_profile", None)
        )
        checkerboard_reasons = list(intrinsics_reasons)
        if checkerboard_profile is None:
            if measurement_active:
                checkerboard_reasons.append("frozen checkerboard Draft missing")
            elif getattr(
                self.panel, "checkerboard_extrinsics_candidate_profile", None
            ) is not None:
                checkerboard_reasons.append("checkerboard stereo R,t candidate not applied")
            else:
                checkerboard_reasons.append("checkerboard stereo R,t missing")
        statuses.append(
            MeasurementMethodStatus(
                method_id=CHECKERBOARD_STEREO,
                instance_id=(
                    "frozen-draft:checkerboard:missing"
                    if checkerboard_profile is None
                    else str(
                        checkerboard_row.get("instance_id")
                        or f"profile:{checkerboard_profile.profile_id or checkerboard_profile.content_sha256}"
                    )
                ),
                ready=not checkerboard_reasons,
                reasons=tuple(checkerboard_reasons),
                profile_id="" if checkerboard_profile is None else checkerboard_profile.profile_id,
                geometry_sha256=(
                    ""
                    if checkerboard_profile is None or not current_profile_hash
                    else self._geometry_evidence_sha256(
                        operating_intrinsics_profile_sha256=current_profile_hash,
                        stereo_extrinsics_sha256=checkerboard_profile.content_sha256,
                    )
                ),
                observed_dofs=POSE_DOF_NAMES,
                component_sources=tuple(
                    (f"{side}.{name}", "checkerboard")
                    for side in ("left", "right")
                    for name in POSE_DOF_NAMES
                ),
            )
        )

        active_selection = getattr(owner, "_active_selection", None)
        active_candidate_id = str(
            getattr(active_selection, "candidate_id", "") or ""
        )
        active_selected_dofs = set(
            getattr(
                getattr(active_selection, "selected_dofs", None),
                "selected_names",
                (),
            )
            or ()
        )
        for candidate in getattr(owner, "_candidates", {}).values():
            method_id = measurement_method_id_for_candidate(candidate.method)
            if method_id is None:
                continue
            frozen_row = frozen_inputs.get(method_id)
            frozen_row = frozen_row if isinstance(frozen_row, Mapping) else {}
            frozen_candidate_id = str(frozen_row.get("candidate_id") or "")
            if frozen_candidate_id and candidate.candidate_id != frozen_candidate_id:
                continue
            if not frozen_candidate_id and measurement_active:
                continue
            candidate_is_globally_active = candidate.candidate_id == active_candidate_id
            candidate_is_frozen = candidate.candidate_id == frozen_candidate_id
            candidate_is_active = candidate_is_globally_active or candidate_is_frozen
            observed_by_side = {
                side: self._candidate_observed_dofs(candidate, side)
                for side in ("left", "right")
            }
            if candidate_is_globally_active:
                observed_by_side = {
                    side: tuple(
                        name
                        for name in observed_by_side[side]
                        if name in active_selected_dofs
                    )
                    for side in ("left", "right")
                }
            observed = tuple(
                name
                for name in POSE_DOF_NAMES
                if all(name in observed_by_side[side] for side in ("left", "right"))
            )
            requires_live_pixhawk = False
            requires_live_gnss = False
            expected_profile_hash = dict(candidate.input_hashes).get(
                "operating_intrinsics_profile_sha256", ""
            )
            reasons: list[str] = list(intrinsics_reasons)
            candidate_warnings = list(candidate.warnings)
            if not candidate_is_active:
                reasons.append(
                    "frozen comparison Draft missing"
                    if measurement_active
                    else "optical pose candidate not applied"
                )
            if not expected_profile_hash:
                reasons.append("candidate operating K,D profile SHA-256 missing")
            elif expected_profile_hash != current_profile_hash:
                reasons.append("candidate K,D differs from active K,D")
            if method_id == HORIZON_HYBRID:
                for side in ("left", "right"):
                    if not {"roll_deg", "pitch_deg"} <= set(observed_by_side[side]):
                        reasons.append(f"{side} Horizon Roll/Pitch not observable")
            elif method_id == DRONE_RTK_ROTATION_HYBRID:
                for side in ("left", "right"):
                    if "yaw_deg" not in observed_by_side[side]:
                        reasons.append(f"{side} Drone RTK rotation not observable")
            elif method_id in {
                MOUNTAIN_GNSS,
                DRONE_RTK_MULTIVIEW,
                TERRAIN_PNP,
                COMBINED_HYBRID,
            }:
                for side in ("left", "right"):
                    missing = set(POSE_DOF_NAMES) - set(observed_by_side[side])
                    if missing:
                        reasons.append(
                            f"{side} full 6-DOF incomplete: {','.join(sorted(missing))}"
                        )

            fillable_method = method_id in {HORIZON_HYBRID, DRONE_RTK_ROTATION_HYBRID}
            fill_sources: dict[str, str] = {}
            for side in MEASUREMENT_POSE_SIDES:
                for dof in MEASUREMENT_POSE_DOFS:
                    key = f"{side}.{dof}"
                    if dof in observed_by_side[side]:
                        fill_sources[key] = f"candidate:{candidate.candidate_id}"
                        continue
                    if not fillable_method:
                        fill_sources[key] = MEASUREMENT_FILL_NONE
                        continue
                    source = self._measurement_fill_source(method_id, side, dof)
                    fill_sources[key] = source
                    compatible = compatible_measurement_fill_source(dof)
                    if source != compatible:
                        reasons.append(f"{key} has no compatible fill source")
                    elif source == MEASUREMENT_FILL_LIVE_PIXHAWK:
                        requires_live_pixhawk = True
                    elif source == MEASUREMENT_FILL_LIVE_GNSS:
                        requires_live_gnss = True
            if requires_live_pixhawk and not pixhawk_ready:
                reasons.append("fresh live Pixhawk required for missing rotation axes")
                reasons.extend(reason for reason in live_hard_reasons if "Pixhawk" in reason)
            if requires_live_gnss and not gnss_ready:
                reasons.append("usable live GNSS required for missing translation axes")
                reasons.extend(reason for reason in live_hard_reasons if "Pixhawk" not in reason)
            component_sources = tuple(
                (
                    f"{side}.{name}",
                    fill_sources[f"{side}.{name}"],
                )
                for side in ("left", "right")
                for name in POSE_DOF_NAMES
            )
            statuses.append(
                MeasurementMethodStatus(
                    method_id=method_id,
                    instance_id=(
                        str(frozen_row.get("instance_id") or "")
                        or f"candidate:{candidate.candidate_id}"
                    ),
                    ready=not reasons,
                    reasons=tuple(reasons),
                    candidate_id=candidate.candidate_id,
                    geometry_sha256=hashlib.sha256(
                        json.dumps(
                            {
                                "candidate_id": candidate.candidate_id,
                                "active_intrinsics": current_profile_hash,
                                "components": component_sources,
                            },
                            sort_keys=True,
                            separators=(",", ":"),
                        ).encode()
                    ).hexdigest(),
                    created_utc_ns=int(candidate.created_utc_ns or 0),
                    observed_dofs=observed,
                    component_sources=component_sources,
                    warnings=tuple(dict.fromkeys((*candidate_warnings, *live_advisories))),
                    requires_live_pixhawk=requires_live_pixhawk,
                    requires_live_gnss=requires_live_gnss,
                )
            )
        loaded_profile = getattr(self.panel, "optical_pose_extrinsics_profile", None)
        loaded_method_id = (
            None
            if loaded_profile is None
            else measurement_method_id_for_candidate(loaded_profile.method)
        )
        if (
            not measurement_active
            and loaded_profile is not None
            and loaded_method_id is not None
        ):
            loaded_instance = f"profile:{loaded_profile.profile_id or loaded_profile.content_sha256}"
            if not any(row.instance_id == loaded_instance for row in statuses):
                reasons = list(intrinsics_reasons)
                loaded_metadata = getattr(loaded_profile, "metadata", {})
                loaded_metadata = (
                    loaded_metadata if isinstance(loaded_metadata, Mapping) else {}
                )
                expected_profile_hash = str(
                    loaded_metadata.get(
                        "operating_intrinsics_profile_sha256",
                        "",
                    )
                    or ""
                )
                loaded_warnings: list[str] = []
                if not expected_profile_hash:
                    reasons.append("applied optical profile operating K,D SHA-256 missing")
                elif expected_profile_hash != current_profile_hash:
                    reasons.append("applied optical profile K,D differs from active K,D")
                stable_reasons = tuple(dict.fromkeys(reasons))
                statuses.append(
                    MeasurementMethodStatus(
                        method_id=loaded_method_id,
                        instance_id=loaded_instance,
                        ready=not stable_reasons,
                        reasons=stable_reasons,
                        profile_id=loaded_profile.profile_id,
                        geometry_sha256=(
                            ""
                            if not current_profile_hash
                            else self._geometry_evidence_sha256(
                                operating_intrinsics_profile_sha256=current_profile_hash,
                                stereo_extrinsics_sha256=loaded_profile.content_sha256,
                            )
                        ),
                        observed_dofs=POSE_DOF_NAMES,
                        component_sources=tuple(
                            (f"{side}.{name}", f"profile:{loaded_profile.profile_id}")
                            for side in ("left", "right")
                            for name in POSE_DOF_NAMES
                        ),
                        warnings=tuple(dict.fromkeys((*loaded_warnings, *live_advisories))),
                    )
                )
        return tuple(statuses)

    def _measurement_fallback_order(self) -> tuple[str, ...]:
        self._measurement_method_state()
        return normalize_measurement_method_fallback_order(
            getattr(
                self.panel,
                "_measurement_active_fallback_order",
                DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
            )
        )

    def _measurement_fill_policy_locked(self) -> bool:
        state = str(getattr(self.panel, "_measurement_package_state", "IDLE") or "IDLE").upper()
        measurement_id = str(getattr(self.panel, "_measurement_id", "") or "")
        detection_started = bool(
            getattr(self.panel, "_measurement_package_detection_started", False)
        )
        return not (measurement_id and state == "COLLECTING" and not detection_started)

    def _measurement_component_controls(
        self,
        status: MeasurementMethodStatus,
    ) -> dict[str, dict[str, object]]:
        sources = dict(status.component_sources)
        pixhawk_ready, gnss_ready, _hard_reasons, _advisories = self._measurement_live_readiness()
        locked = self._measurement_fill_policy_locked()
        rows: dict[str, dict[str, object]] = {}
        atomic_method = status.method_id in {
            PIXHAWK_RTK_LIVE,
            PIXHAWK_RP_GPS_BASELINE_YAW,
            CHECKERBOARD_STEREO,
        } or bool(
            status.profile_id and not status.candidate_id
        )
        for side in MEASUREMENT_POSE_SIDES:
            for dof in MEASUREMENT_POSE_DOFS:
                key = f"{side}.{dof}"
                source = str(sources.get(key) or MEASUREMENT_FILL_NONE)
                candidate_provided = source.startswith("candidate:")
                provided = bool(atomic_method or candidate_provided or source.startswith("profile:"))
                essential_missing = bool(
                    status.candidate_id
                    and (
                        status.method_id == HORIZON_HYBRID
                        and dof in {"roll_deg", "pitch_deg"}
                        or status.method_id == DRONE_RTK_ROTATION_HYBRID
                        and dof == "yaw_deg"
                        or status.method_id
                        in {MOUNTAIN_GNSS, DRONE_RTK_MULTIVIEW, TERRAIN_PNP, COMBINED_HYBRID}
                    )
                    and not candidate_provided
                )
                compatible = compatible_measurement_fill_source(dof)
                editable = bool(not provided and not essential_missing)
                if editable:
                    source = self._measurement_fill_source(status.method_id, side, dof)
                source_ready = (
                    pixhawk_ready
                    if compatible == MEASUREMENT_FILL_LIVE_PIXHAWK
                    else gnss_ready
                )
                rows[key] = {
                    "side": side,
                    "dof": dof,
                    "provided": provided,
                    "editable": editable,
                    "locked": bool(locked or not editable),
                    "source": source,
                    "compatible_fill_source": compatible,
                    "fill_enabled": bool(editable and source == compatible),
                    "source_ready": bool(source_ready),
                    "reason": (
                        "provided by method"
                        if provided
                        else "method-defining DOF is not observable"
                        if essential_missing
                        else "fill source selected"
                        if source == compatible
                        else "fill disabled"
                    ),
                }
        return rows

    def configure_workstation_measurement_method_fill(
        self,
        payload: object,
    ) -> dict[str, object]:
        self._measurement_method_state()
        if self._measurement_fill_policy_locked():
            raise RuntimeError("DOF fill policy is editable only before Detection starts")
        values = dict(payload) if isinstance(payload, Mapping) else {}
        method_id = normalize_measurement_method_id(values.get("method_id"))
        component = str(values.get("component") or "").strip().lower()
        if "." not in component:
            raise ValueError("component must be '<side>.<dof>'")
        side, dof = component.split(".", 1)
        if side not in MEASUREMENT_POSE_SIDES or dof not in MEASUREMENT_POSE_DOFS:
            raise ValueError(f"invalid measurement fill component: {component}")
        statuses = latest_status_by_method(self._measurement_method_statuses())
        status = statuses.get(method_id)
        if status is None:
            raise ValueError(f"measurement method is unavailable: {method_id}")
        control = self._measurement_component_controls(status)[component]
        if not bool(control["editable"]):
            raise ValueError(f"measurement component is method-owned: {method_id}/{component}")
        enabled = bool(values.get("enabled", True))
        source = compatible_measurement_fill_source(dof) if enabled else MEASUREMENT_FILL_NONE
        policy = self._measurement_fill_policy()
        policy[method_id][component] = source
        self.panel._measurement_method_fill_policy = normalize_measurement_method_fill_policy(policy)

        package = getattr(self.panel, "_measurement_package", None)
        updater = getattr(package, "update_metadata", None)
        if callable(updater):
            updater({"measurement_method_fill_policy": self.panel._measurement_method_fill_policy})
        writer = getattr(self.panel, "append_workstation_run_event", None)
        if callable(writer):
            writer(
                "measurement_method_fill_changed",
                {
                    "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
                    "method_id": method_id,
                    "component": component,
                    "source": source,
                },
                source="MeasurementViewController",
                coordinate_contract="per-Measurement virtual-FRD RPY / NED center fill policy",
            )
        self.panel._measurement_selected_method_id = ""
        self.panel._measurement_selected_instance_id = ""
        self.panel._measurement_selection_reason = "fill_policy_changed"
        self.panel._measurement_method_ready_since = {}
        return self.workstation_measurement_methods_snapshot()

    def _set_selected_measurement_method(
        self,
        status: MeasurementMethodStatus,
        *,
        reason: str,
    ) -> None:
        self._measurement_method_state()
        panel = self.panel
        previous_method = panel._measurement_selected_method_id
        previous_instance = panel._measurement_selected_instance_id
        if previous_method == status.method_id and previous_instance == status.instance_id:
            return
        panel._measurement_selected_method_id = status.method_id
        panel._measurement_selected_instance_id = status.instance_id
        panel._measurement_selection_reason = str(reason)
        panel._measurement_method_transition_sequence += 1
        panel._last_measurement_attempt_key = None
        panel._last_measurement_attempt = None
        writer = getattr(panel, "append_workstation_run_event", None)
        if callable(writer):
            writer(
                "measurement_method_switched",
                {
                    "sequence": panel._measurement_method_transition_sequence,
                    "from_method_id": previous_method,
                    "from_instance_id": previous_instance,
                    "to_method_id": status.method_id,
                    "to_instance_id": status.instance_id,
                    "reason": reason,
                    "fallback_order": list(self._measurement_fallback_order()),
                    "component_sources": dict(status.component_sources),
                    "source_identity_sha256": status.geometry_sha256,
                },
                source="MeasurementViewController",
                coordinate_contract="measurement method selection; geometry frozen per pair",
            )

    def _clear_selected_measurement_method(self, *, reason: str) -> None:
        self._measurement_method_state()
        panel = self.panel
        previous_method = panel._measurement_selected_method_id
        previous_instance = panel._measurement_selected_instance_id
        if not previous_method and not previous_instance:
            panel._measurement_selection_reason = str(reason)
            return
        panel._measurement_selected_method_id = ""
        panel._measurement_selected_instance_id = ""
        panel._measurement_selection_reason = str(reason)
        panel._measurement_method_transition_sequence += 1
        panel._last_measurement_attempt_key = None
        panel._last_measurement_attempt = None
        writer = getattr(panel, "append_workstation_run_event", None)
        if callable(writer):
            writer(
                "measurement_method_unavailable",
                {
                    "sequence": panel._measurement_method_transition_sequence,
                    "from_method_id": previous_method,
                    "from_instance_id": previous_instance,
                    "reason": reason,
                    "fallback_order": list(self._measurement_fallback_order()),
                },
                source="MeasurementViewController",
                coordinate_contract="measurement method unavailable; no geometry selected",
            )

    def _select_measurement_method_for_attempt(self) -> MeasurementMethodStatus | None:
        self._measurement_method_state()
        panel = self.panel
        statuses = self._measurement_method_statuses()
        now_s = time.monotonic()
        ready_keys = {
            (row.method_id, row.instance_id) for row in statuses if row.ready
        }
        ready_since = panel._measurement_method_ready_since
        for key in tuple(ready_since):
            if key not in ready_keys:
                ready_since.pop(key, None)
        for key in ready_keys:
            ready_since.setdefault(key, now_s)
        current = next(
            (
                row
                for row in statuses
                if row.method_id == panel._measurement_selected_method_id
                and row.instance_id == panel._measurement_selected_instance_id
            ),
            None,
        )
        preferred = panel._measurement_preferred_method_id
        chosen = choose_ready_measurement_method(
            statuses,
            preferred_method_id=preferred,
            fallback_order=self._measurement_fallback_order(),
        )
        if chosen is None:
            self._clear_selected_measurement_method(reason="no_ready_method")
            return None
        if current is None or not current.ready:
            reason = "preferred" if chosen.method_id == preferred else "auto_fallback"
            self._set_selected_measurement_method(chosen, reason=reason)
            return chosen
        if (current.method_id, current.instance_id) == (chosen.method_id, chosen.instance_id):
            return current
        key = (chosen.method_id, chosen.instance_id)
        since = float(ready_since[key])
        if now_s - since < 1.0:
            return current
        reason = "auto_recovery" if chosen.method_id == preferred else "priority_preemption"
        self._set_selected_measurement_method(chosen, reason=reason)
        return chosen

    def _measurement_method_input_snapshot(
        self,
        status: MeasurementMethodStatus,
    ) -> dict[str, object]:
        """Expose the factual values currently feeding one method.

        This is a read-only operator projection.  It deliberately keeps live
        PX4 body-FRD/world-NED attitude, the P1->P2 GNSS NED baseline, absolute
        candidate poses, and OpenCV left-to-right stereo R,t in separate
        fields instead of presenting unlike coordinate contracts as one pose.
        """

        now_s = time.monotonic()
        component_sources = dict(status.component_sources)
        sources = set(component_sources.values())
        attitude_rows: dict[str, object] = {}
        live_attitudes = getattr(self.panel, "latest_extrinsics", {}) or {}
        for side in MEASUREMENT_POSE_SIDES:
            sample = live_attitudes.get(side) if isinstance(live_attitudes, Mapping) else None
            if sample is None:
                attitude_rows[side] = None
                continue
            timestamp_s = getattr(sample, "timestamp_s", None)
            try:
                age_s = max(0.0, now_s - float(timestamp_s))
            except (TypeError, ValueError):
                age_s = None
            attitude_rows[side] = {
                "roll_deg": getattr(sample, "roll_deg", None),
                "pitch_deg": getattr(sample, "pitch_deg", None),
                "yaw_deg": getattr(sample, "yaw_deg", None),
                "age_s": age_s,
                "source": [
                    str(getattr(sample, "namespace", "") or ""),
                    str(getattr(sample, "topic", "") or ""),
                ],
            }

        baseline = getattr(self.panel, "latest_rtk_baseline", None)
        baseline_row: object = None
        if baseline is not None:
            baseline_row = {
                "north_m": getattr(baseline, "north_m", None),
                "east_m": getattr(baseline, "east_m", None),
                "down_m": getattr(baseline, "down_m", None),
                "length_m": getattr(baseline, "baseline_m", None),
                "left_age_s": getattr(baseline, "left_age_s", None),
                "right_age_s": getattr(baseline, "right_age_s", None),
                "fix_quality": str(getattr(baseline, "fix_quality", "") or ""),
                "warnings": list(getattr(baseline, "warnings", ()) or ()),
            }

        candidate_rows: dict[str, object] = {}
        candidate = None
        if status.candidate_id:
            try:
                candidate = self._measurement_candidate(status.candidate_id)
            except (AttributeError, RuntimeError, TypeError, ValueError):
                # Lightweight/legacy facades may expose a persisted status
                # without the live optical-pose owner.  The operator snapshot
                # must show the missing value, not break fill configuration.
                candidate = None
        if candidate is not None:
            for side in MEASUREMENT_POSE_SIDES:
                pose = getattr(candidate, f"{side}_pose_ned", None)
                if pose is None:
                    candidate_rows[side] = None
                    continue
                try:
                    values = pose_parameters_from_camera_pose(pose)
                except (AttributeError, TypeError, ValueError):
                    candidate_rows[side] = None
                    continue
                observed_values = {
                    name: getattr(values, name)
                    for name in POSE_DOF_NAMES
                    if str(component_sources.get(f"{side}.{name}") or "").startswith(
                        "candidate:"
                    )
                }
                candidate_rows[side] = observed_values or None

        profile = None
        if status.method_id == CHECKERBOARD_STEREO:
            profile = getattr(self.panel, "checkerboard_extrinsics_profile", None)
        elif status.profile_id and not status.candidate_id:
            loaded = getattr(self.panel, "optical_pose_extrinsics_profile", None)
            if loaded is not None and str(getattr(loaded, "profile_id", "")) == status.profile_id:
                profile = loaded
        extrinsics_row: object = None
        extrinsics = None if profile is None else getattr(profile, "extrinsics", None)
        if extrinsics is not None:
            extrinsics_row = {
                "rotation_left_to_right": getattr(extrinsics, "rotation_left_to_right", None),
                "translation_left_to_right_m": getattr(
                    extrinsics,
                    "translation_left_to_right_m",
                    None,
                ),
                "frame_contract": str(getattr(extrinsics, "frame_contract", "") or ""),
                "profile_id": str(getattr(profile, "profile_id", "") or ""),
            }

        baseline_yaw_row: object = None
        if status.method_id == PIXHAWK_RP_GPS_BASELINE_YAW:
            selection, p1_age_s, p2_age_s = self._baseline_yaw_selection_preview()
            if selection is not None:
                baseline_yaw_row = {
                    **selection.to_mapping(),
                    "p1_age_s": p1_age_s,
                    "p2_age_s": p2_age_s,
                }

        return {
            "uses_live_pixhawk": MEASUREMENT_FILL_LIVE_PIXHAWK in sources,
            "uses_live_gnss": MEASUREMENT_FILL_LIVE_GNSS in sources,
            "live_attitude_rpy_deg": attitude_rows,
            "live_baseline_ned_m": baseline_row,
            "candidate_pose_virtual_frd_to_world_ned": candidate_rows,
            "stereo_extrinsics_opencv_rdf": extrinsics_row,
            "baseline_yaw_selection": baseline_yaw_row,
        }

    def workstation_measurement_methods_snapshot(self) -> dict[str, object]:
        self._measurement_method_state()
        frozen_measurement = bool(
            getattr(
                self.panel,
                "_measurement_frozen_comparison_inputs_declared",
                False,
            )
        )
        selected = (
            latest_status_by_method(self._measurement_method_statuses()).get(
                normalize_measurement_method_id(
                    getattr(self.panel, "_measurement_selected_method_id", "")
                    or PIXHAWK_RTK_LIVE
                )
            )
            if frozen_measurement
            else self._select_measurement_method_for_attempt()
        )
        statuses = self._measurement_method_statuses()
        latest = latest_status_by_method(statuses)
        ready = [row for row in latest.values() if row.ready]
        order = self._measurement_fallback_order()
        rank = {method_id: index + 1 for index, method_id in enumerate(order)}
        ready.sort(key=lambda row: (rank.get(row.method_id, len(rank) + 1), row.label))
        capability_rows = {
            method_id: {
                **row.to_mapping(),
                "components": self._measurement_component_controls(row),
                "input_values": self._measurement_method_input_snapshot(row),
                "required_inputs": list(MEASUREMENT_METHOD_REQUIRED_INPUTS[method_id]),
            }
            for method_id, row in latest.items()
        }
        method_catalog = []
        last_bundle = getattr(self.panel, "_last_method_evaluation_bundle", None)
        last_outcomes = (
            {row.method_id: row for row in last_bundle.outcomes}
            if isinstance(last_bundle, MethodEvaluationBundle)
            else {}
        )
        counters = getattr(self.panel, "_measurement_method_result_counters", {})
        for method_id in MEASUREMENT_METHOD_IDS:
            capability = capability_rows.get(method_id)
            if capability is not None:
                row = dict(capability)
            else:
                row = {
                    "method_id": method_id,
                    "label": MEASUREMENT_METHOD_LABELS[method_id],
                    "instance_id": "",
                    "ready": False,
                    "reasons": [MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS[method_id]],
                    "candidate_id": "",
                    "profile_id": "",
                    "geometry_sha256": "",
                    "source_identity_sha256": "",
                    "created_utc_ns": 0,
                    "observed_dofs": [],
                    "component_sources": {},
                    "warnings": [],
                    "requires_live_pixhawk": False,
                    "requires_live_gnss": False,
                    "components": {},
                    "input_values": {},
                    "required_inputs": list(MEASUREMENT_METHOD_REQUIRED_INPUTS[method_id]),
                }
            outcome = last_outcomes.get(method_id)
            if outcome is not None:
                runtime_status = (
                    "REUSED"
                    if outcome.reused_inputs
                    and outcome.status in {"VALID", "ESTIMATED"}
                    else "ACTIVE"
                    if outcome.status in {"VALID", "ESTIMATED"}
                    else "BLOCKED"
                    if outcome.status in {"BLOCKED", "BACKPRESSURE_BLOCKED"}
                    else outcome.status
                )
                runtime_reasons = list(outcome.reasons)
            else:
                runtime_status = (
                    "ACTIVE"
                    if frozen_measurement and bool(row.get("ready"))
                    else "READY"
                    if bool(row.get("ready"))
                    else "BLOCKED"
                )
                runtime_reasons = list(row.get("reasons") or ())
            row["runtime_status"] = runtime_status
            row["primary"] = method_id == PIXHAWK_RTK_LIVE
            row["frozen_for_measurement"] = frozen_measurement
            row["primary_blocker"] = runtime_reasons[0] if runtime_reasons else ""
            row["additional_blocker_count"] = max(0, len(runtime_reasons) - 1)
            row["result_counters"] = dict(
                counters.get(
                    method_id,
                    {"VALID": 0, "BLOCKED": 0, "FAILED": 0},
                )
            )
            method_catalog.append(row)
        return {
            "schema": "measurement-method-snapshot-v1",
            "preferred_method_id": self.panel._measurement_preferred_method_id,
            "selected_method_id": "" if selected is None else selected.method_id,
            "selected_instance_id": "" if selected is None else selected.instance_id,
            "selection_reason": self.panel._measurement_selection_reason,
            "transition_sequence": self.panel._measurement_method_transition_sequence,
            "fallback_order": list(order),
            "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
            "fill_policy_locked": self._measurement_fill_policy_locked(),
            "fill_policy": self._measurement_fill_policy(),
            "method_capabilities": list(capability_rows.values()),
            "method_catalog": method_catalog,
            "ready_methods": [
                {**row.to_mapping(), "fallback_rank": rank[row.method_id]}
                for row in ready
            ],
            "all_methods": [row.to_mapping() for row in statuses],
        }

    def select_workstation_measurement_method(self, payload: object) -> dict[str, object]:
        self._measurement_method_state()
        values = payload if isinstance(payload, dict) else {"method_id": payload}
        method_id = normalize_measurement_method_id(values.get("method_id"))
        ready = {
            row.method_id: row
            for row in latest_status_by_method(self._measurement_method_statuses()).values()
            if row.ready
        }
        status = ready.get(method_id)
        if status is None:
            raise ValueError(f"measurement method is not ready: {method_id}")
        self.panel._measurement_preferred_method_id = method_id
        package = getattr(self.panel, "_measurement_package", None)
        updater = getattr(package, "update_metadata", None)
        if callable(updater):
            updater({"measurement_preferred_method": method_id})
        self._set_selected_measurement_method(status, reason="manual")
        return self.workstation_measurement_methods_snapshot()

    def _measurement_candidate(self, candidate_id: str):
        owner = self.panel._optical_pose_controller()
        return getattr(owner, "_candidates", {}).get(str(candidate_id))

    def _measurement_intrinsics_result_for_method(
        self,
        status: MeasurementMethodStatus,
        *,
        left_attitude: AttitudeSnapshot | None = None,
        right_attitude: AttitudeSnapshot | None = None,
        left_gps: GpsSnapshot | None = None,
        right_gps: GpsSnapshot | None = None,
    ) -> StereoIntrinsicsResult | None:
        owner = self.panel._optical_pose_controller()
        if bool(
            getattr(
                self.panel,
                "_measurement_frozen_comparison_inputs_declared",
                False,
            )
        ):
            intrinsics_only = getattr(
                self.panel, "_pending_measurement_frozen_intrinsics_result", None
            )
            if intrinsics_only is None:
                return None
        else:
            current = getattr(self.panel, "active_intrinsics", None)
            profile_reader = getattr(
                owner, "operating_intrinsics_profile_for_geometry", None
            )
            profile = (
                profile_reader()
                if callable(profile_reader)
                else owner._operating_intrinsics_profile()
            )
            if profile is None:
                return None
            if profile.provenance.get("quality") == NOMINAL_INTRINSICS_QUALITY:
                current = nominal_profile_to_stereo_result(profile)
            elif current is None:
                return None
            intrinsics_only = without_stereo_extrinsics(current)
        if status.method_id in {PIXHAWK_RTK_LIVE, PIXHAWK_RP_GPS_BASELINE_YAW}:
            return intrinsics_only
        if status.method_id == CHECKERBOARD_STEREO:
            frozen_objects = getattr(
                self.panel, "_pending_measurement_frozen_comparison_objects", {}
            )
            profile = (
                frozen_objects.get(CHECKERBOARD_STEREO)
                if bool(
                    getattr(
                        self.panel,
                        "_measurement_frozen_comparison_inputs_declared",
                        False,
                    )
                )
                and isinstance(frozen_objects, Mapping)
                else getattr(self.panel, "checkerboard_extrinsics_profile", None)
            )
            return intrinsics_only if profile is None else compose_stereo_calibration(intrinsics_only, profile)
        if status.profile_id and not status.candidate_id:
            profile = getattr(self.panel, "optical_pose_extrinsics_profile", None)
            if profile is not None and profile.profile_id == status.profile_id:
                return compose_stereo_calibration(intrinsics_only, profile)
        candidate = self._measurement_candidate(status.candidate_id)
        if candidate is None:
            return None
        owner = self.panel._optical_pose_controller()
        priors = getattr(owner, "_candidate_priors", {}).get(candidate.candidate_id)
        if not isinstance(priors, tuple) or len(priors) != 2:
            return None
        full_candidate_methods = {
            MOUNTAIN_GNSS,
            DRONE_RTK_MULTIVIEW,
            TERRAIN_PNP,
            COMBINED_HYBRID,
        }
        if status.method_id in full_candidate_methods:
            poses = [candidate.left_pose_ned, candidate.right_pose_ned]
        else:
            if left_gps is None or right_gps is None:
                return None
            baseline_ned = ned_delta_m(left_gps, right_gps)
            if baseline_ned is None:
                return None
            left_anchor = tuple(priors[0].center_ned_m)
            live_centers = {
                "left": left_anchor,
                "right": tuple(
                    left_anchor[index] + float(baseline_ned[index])
                    for index in range(3)
                ),
            }
            attitude_by_side = {"left": left_attitude, "right": right_attitude}
            drone_context = getattr(owner, "_candidate_drone_rotation_context", {}).get(
                candidate.candidate_id, {}
            )
            poses = []
            for side, candidate_pose in zip(
                ("left", "right"),
                (candidate.left_pose_ned, candidate.right_pose_ned),
            ):
                attitude = attitude_by_side[side]
                candidate_values = pose_parameters_from_camera_pose(candidate_pose)
                if status.method_id == HORIZON_HYBRID:
                    if attitude is None:
                        return None
                    final_values = PoseParametersNed(
                        candidate_values.roll_deg,
                        candidate_values.pitch_deg,
                        attitude.yaw_deg,
                        *live_centers[side],
                    )
                elif status.method_id == DRONE_RTK_ROTATION_HYBRID:
                    observed = set(self._candidate_observed_dofs(candidate, side))
                    if {"roll_deg", "pitch_deg", "yaw_deg"} <= observed:
                        final_values = PoseParametersNed(
                            candidate_values.roll_deg,
                            candidate_values.pitch_deg,
                            candidate_values.yaw_deg,
                            *live_centers[side],
                        )
                    else:
                        context = drone_context.get(side, {})
                        if attitude is None or context.get("mode") != "yaw_only":
                            return None
                        estimate = estimate_drone_yaw_deg(
                            context.get("observed_ray_camera_rdf", ()),
                            context.get("expected_los_ned", ()),
                            roll_deg=attitude.roll_deg,
                            pitch_deg=attitude.pitch_deg,
                        )
                        if not estimate.observable or estimate.yaw_deg is None:
                            return None
                        final_values = PoseParametersNed(
                            attitude.roll_deg,
                            attitude.pitch_deg,
                            estimate.yaw_deg,
                            *live_centers[side],
                        )
                else:
                    return None
                poses.append(final_values.to_camera_pose_ned())
        stereo = stereo_pose_from_absolute(poses[0], poses[1])
        candidate_quality = dict(candidate.quality_metrics)
        stereo_extrinsics = StereoExtrinsics(
            rotation_left_to_right=[list(row) for row in stereo.rotation_right_from_left_rdf],
            translation_left_to_right_m=list(stereo.translation_right_from_left_rdf_m),
            reprojection_error_px=float(candidate_quality.get("reprojection_error_px", 0.0)),
            pair_count=len(candidate.source_observation_ids),
        )
        profile = StereoExtrinsicsProfile(
            extrinsics=stereo_extrinsics,
            source_zoom_level=getattr(self.panel, "_active_intrinsics_source_zoom_level", None),
            method=f"measurement/{candidate.method}",
            source_intrinsics_path=str(getattr(self.panel, "_active_intrinsics_profile_path", "") or ""),
            source_intrinsics_sha256=intrinsics_content_sha256(intrinsics_only),
            profile_id=status.instance_id,
            metadata={
                "measurement_method_id": status.method_id,
                "candidate_id": candidate.candidate_id,
                "component_sources": dict(status.component_sources),
            },
        )
        return compose_stereo_calibration(intrinsics_only, profile)

    @staticmethod
    def _measurement_geometry_hash(
        status: MeasurementMethodStatus,
        *,
        geometry_kind: str,
        intrinsics: StereoIntrinsicsSnapshot | None,
        left_attitude: AttitudeSnapshot | None,
        right_attitude: AttitudeSnapshot | None,
        left_gps: GpsSnapshot | None,
        right_gps: GpsSnapshot | None,
        geometry_left_quaternion: tuple[float, float, float, float] | None = None,
        geometry_right_quaternion: tuple[float, float, float, float] | None = None,
        yaw_selection: BaselineYawSelection | None = None,
    ) -> str:
        component_values = set(dict(status.component_sources).values())

        def attitude_payload(value: AttitudeSnapshot | None) -> object:
            if value is None or "live_pixhawk" not in component_values:
                return None
            return {
                "timestamp_s": value.timestamp_s,
                "q_wxyz": list(value.quaternion_wxyz),
                "rpy_deg": [value.roll_deg, value.pitch_deg, value.yaw_deg],
                "source": [value.namespace, value.topic],
            }

        def gps_payload(value: GpsSnapshot | None) -> object:
            if value is None or "live_gnss" not in component_values:
                return None
            return {
                "timestamp_s": value.timestamp_s,
                "latitude_deg": value.latitude_deg,
                "longitude_deg": value.longitude_deg,
                "altitude_m": value.altitude_m,
                "fix_type": value.fix_type,
                "source": [value.namespace, value.topic],
            }

        payload = {
            "schema": "resolved-measurement-geometry-v1",
            "method_id": status.method_id,
            "instance_id": status.instance_id,
            "geometry_kind": geometry_kind,
            "intrinsics_and_relative_pose_sha256": (
                "" if intrinsics is None else intrinsics.content_sha256
            ),
            "component_sources": dict(status.component_sources),
            "left_attitude": attitude_payload(left_attitude),
            "right_attitude": attitude_payload(right_attitude),
            "geometry_left_quaternion_wxyz": (
                None if geometry_left_quaternion is None else list(geometry_left_quaternion)
            ),
            "geometry_right_quaternion_wxyz": (
                None if geometry_right_quaternion is None else list(geometry_right_quaternion)
            ),
            "baseline_yaw_selection": (
                None if yaw_selection is None else yaw_selection.to_mapping()
            ),
            "left_gps": gps_payload(left_gps),
            "right_gps": gps_payload(right_gps),
        }
        return hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()

    def _record_no_ready_measurement_pair(self, pair) -> None:
        timing = self._pair_timing_decision(pair)
        timestamp_basis = timing.effective_basis
        left_pairing_timestamp_s = timing.left_timestamp_s
        right_pairing_timestamp_s = timing.right_timestamp_s
        statuses = self._measurement_method_statuses()
        readiness_payload = {
            "schema": "measurement-readiness-block-v1",
            "preferred_method_id": str(
                getattr(self.panel, "_measurement_preferred_method_id", "") or ""
            ),
            "fallback_order": list(self._measurement_fallback_order()),
            "methods": [row.to_mapping() for row in statuses],
        }
        readiness_sha256 = hashlib.sha256(
            json.dumps(readiness_payload, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        reasons = tuple(
            f"{row.method_id}: {', '.join(row.reasons) or 'not ready'}"
            for row in latest_status_by_method(statuses).values()
            if not row.ready
        ) or ("no ready measurement method",)
        blocked = BlockedMeasurementPair(
            detection_pair_key=detection_pair_key(
                triangulation_source="no_ready_measurement_method",
                timestamp_basis=timestamp_basis,
                left=pair.left,
                right=pair.right,
                left_pairing_timestamp_s=left_pairing_timestamp_s,
                right_pairing_timestamp_s=right_pairing_timestamp_s,
            ),
            created_monotonic_s=time.monotonic(),
            created_wall_s=time.time(),
            timestamp_basis=timestamp_basis,
            left_detection=DetectionSnapshot.from_observation(
                pair.left,
                pairing_timestamp_basis=timestamp_basis,
                pairing_timestamp_s=left_pairing_timestamp_s,
            ),
            right_detection=DetectionSnapshot.from_observation(
                pair.right,
                pairing_timestamp_basis=timestamp_basis,
                pairing_timestamp_s=right_pairing_timestamp_s,
            ),
            status="matched_no_ready_measurement_method",
            reasons=reasons,
            readiness_sha256=readiness_sha256,
            readiness_payload_json=json.dumps(
                readiness_payload,
                sort_keys=True,
                separators=(",", ":"),
            ),
            requested_timestamp_basis=timing.requested_basis,
            timing_fallback_reason=timing.fallback_reason,
        )
        self.panel._record_blocked_measurement_pair(blocked)

    @staticmethod
    def _release_detection_pair_reservation(reservation: object | None) -> None:
        release = getattr(reservation, "release", None)
        if callable(release):
            release()

    def _exact_pair_reservation_supported(self) -> bool:
        return all(
            callable(
                getattr(
                    getattr(self.panel, f"{side}_pane", None),
                    "native_ring_for_frame_key",
                    None,
                )
            )
            for side in ("left", "right")
        )

    @staticmethod
    def _reserve_detection_pair(
        pair: object,
        sources: Mapping[str, object | None],
    ) -> RingPairReservation | None:
        frames = []
        for side, observation in (("left", pair.left), ("right", pair.right)):
            metadata = getattr(observation, "timestamp_metadata", {})
            if not isinstance(metadata, Mapping):
                raise ValueError(f"{side} frame metadata is unavailable")
            key = FrameKey.from_mapping(metadata)
            if (
                key.camera_id != side
                or key.stream_generation <= 0
                or key.frame_id <= 0
            ):
                raise ValueError(f"{side} exact FrameKey is invalid")
            ring = sources.get(side)
            if ring is None:
                raise ValueError(f"{side} exact native ring is unavailable")
            frames.append((side, ring, key, metadata))
        return reserve_ring_pair(frames)

    def _ensure_detection_pair_coordinator(self) -> DetectionPairCoordinator:
        coordinator = self._detection_pair_coordinator
        if coordinator is None:
            coordinator = DetectionPairCoordinator(
                reservation_factory=(
                    self._reserve_detection_pair
                    if self._exact_pair_reservation_supported()
                    else None
                )
            )
            self._detection_pair_coordinator = coordinator
        if (
            self._detection_pair_timer is None
            and QtCore.QCoreApplication.instance() is not None
        ):
            parent = self.panel if isinstance(self.panel, QtCore.QObject) else None
            timer = QtCore.QTimer(parent)
            timer.setInterval(20)
            timer.timeout.connect(self._poll_detection_pairs)
            timer.start()
            self._detection_pair_timer = timer
        return coordinator

    def _configured_time_basis(self) -> str:
        panel = self.panel
        combo = getattr(panel, "yolo_gate_time_basis_combo", None)
        current_data = getattr(combo, "currentData", None)
        if callable(current_data):
            value = current_data()
            if value is not None:
                return normalize_live_distance_time_basis(value)
        settings = getattr(panel, "settings", None)
        default = getattr(
            settings,
            "yolo_gate_time_basis",
            TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC,
        )
        setting_reader = getattr(panel, "_workstation_setting", None)
        value = (
            setting_reader("yolo_gate_time_basis", default)
            if callable(setting_reader)
            else default
        )
        return normalize_live_distance_time_basis(value)

    def requested_time_basis(self) -> str:
        """Return the current Measurement-frozen request or the idle setting."""

        measurement_id = str(getattr(self.panel, "_measurement_id", "") or "")
        if not measurement_id:
            return self._configured_time_basis()
        self._measurement_method_state()
        return normalize_live_distance_time_basis(
            getattr(
                self.panel,
                "_measurement_active_time_basis_requested",
                self._configured_time_basis(),
            )
        )

    def _rtcp_pairing_health(self, requested_basis: str) -> tuple[bool | None, str]:
        if requested_basis != "camera_rtcp":
            return None, ""
        if str(getattr(self.panel, "camera_sync_status", "")) != "verified":
            return False, "CLOCK_NOT_VERIFIED"
        if str(getattr(self.panel, "frame_timing_status", "")) != "current":
            return False, "FRAME_TIMING_NOT_CURRENT"
        return True, ""

    def _pairing_policy_snapshot(self) -> PairingPolicy:
        configured_delta_s = float(self.panel._yolo_pair_max_delta_s())
        app_delta_limit_s = (
            min(LIVE_DISTANCE_MAX_PAIR_DELTA_S, configured_delta_s)
            if math.isfinite(configured_delta_s)
            else LIVE_DISTANCE_MAX_PAIR_DELTA_S
        )
        requested_basis = self.requested_time_basis()
        rtcp_verified, rtcp_reason = self._rtcp_pairing_health(requested_basis)
        measurement_generation = str(
            getattr(self.panel, "_measurement_id", "") or "idle"
        )
        token = (
            measurement_generation,
            requested_basis,
            rtcp_verified,
            rtcp_reason,
            app_delta_limit_s,
            float(self.panel._yolo_vertical_max_delta_px()),
        )
        if token != self._pairing_policy_token:
            self._pairing_policy_token = token
            self._pairing_policy_revision += 1
            previous_reservation = getattr(
                self.panel,
                "_coordinated_detection_pair_reservation",
                None,
            )
            self._release_detection_pair_reservation(previous_reservation)
            self.panel._coordinated_detection_pair_reservation = None
            self.panel._coordinated_detection_pair = None
            self.panel._coordinated_pairing_result = None
            self.panel._last_frozen_measurement_pair = None
        return PairingPolicy(
            timestamp_basis=requested_basis,
            max_delta_s=app_delta_limit_s,
            max_vertical_delta_px=float(self.panel._yolo_vertical_max_delta_px()),
            allow_app_fallback=requested_basis == "camera_rtcp",
            rtcp_clock_verified=rtcp_verified,
            rtcp_unavailable_reason=rtcp_reason,
            require_current_clock_epoch=requested_basis == "camera_rtcp",
            measurement_generation=measurement_generation,
            max_observation_age_s=DETECTION_PAIR_MAX_OBSERVATION_AGE_S,
            revision=self._pairing_policy_revision,
        )

    def _poll_detection_pairs(self) -> None:
        coordinator = self._detection_pair_coordinator
        if coordinator is None:
            return
        result = coordinator.poll_latest()
        if result is None:
            return
        current_policy = self._pairing_policy_snapshot()
        if result.policy_revision != current_policy.revision:
            self._release_detection_pair_reservation(result.reservation)
            self.panel._coordinated_detection_pair = None
            self.panel._coordinated_pairing_result = None
            return
        previous_reservation = getattr(
            self.panel,
            "_coordinated_detection_pair_reservation",
            None,
        )
        if previous_reservation is not result.reservation:
            self._release_detection_pair_reservation(previous_reservation)
        self.panel._coordinated_detection_pair_reservation = result.reservation
        self.panel._detection_pair_coordinator_active = True
        self.panel._coordinated_detection_pair = result.pair
        self.panel._coordinated_pairing_result = result
        self.panel._last_correspondence_rejection_reason = result.rejection_reason
        self.panel._last_pair_advisories = result.advisories
        self.panel._runtime_pair_uses_app_fallback = result.uses_app_fallback
        self.panel._detection_pair_metrics = coordinator.metrics()
        try:
            self.panel._refresh_distance()
            self.panel._refresh_optical_alignment_view()
        finally:
            current_reservation = getattr(
                self.panel,
                "_coordinated_detection_pair_reservation",
                None,
            )
            if current_reservation is result.reservation:
                self._release_detection_pair_reservation(current_reservation)
                self.panel._coordinated_detection_pair_reservation = None

    def reset_detection_pair_coordinator(self) -> None:
        coordinator = self._detection_pair_coordinator
        self._detection_pair_coordinator = None
        if coordinator is not None:
            coordinator.close()
        self.panel._detection_pair_coordinator_active = False
        self.panel._coordinated_detection_pair = None
        self.panel._coordinated_pairing_result = None
        self._release_detection_pair_reservation(
            getattr(self.panel, "_coordinated_detection_pair_reservation", None)
        )
        self.panel._coordinated_detection_pair_reservation = None
        self.panel._detection_pair_metrics = {}
        preview_coordinator = self._preview_measurement_coordinator
        self._preview_measurement_coordinator = None
        if preview_coordinator is not None:
            preview_coordinator.close()
        self._preview_measurement_results.clear()
        self._preview_measurement_pending.clear()
        self.panel._latest_preview_measurement_result = None
        self.panel._latest_preview_pair_identity = None

    def begin_pairing_generation(self) -> None:
        """Retire every pre-Measurement candidate/result before freezing settings."""

        self._pairing_policy_token = None
        self._primary_outcome_hash_by_key.clear()
        self._primary_normalized_outcome_by_key.clear()
        self._primary_measurement_persisted.clear()
        self._primary_history_projected.clear()
        self._primary_display_high_water = -1
        self._primary_displayed_key = None
        self.panel._measurement_recording_method_id = ""
        self.reset_detection_pair_coordinator()
        self.panel._last_pair_advisories = ()
        self.panel._last_correspondence_rejection_reason = ""
        self.panel._runtime_pair_uses_app_fallback = False

    def _ensure_preview_measurement_coordinator(
        self,
    ) -> PreviewMeasurementCoordinator:
        coordinator = self._preview_measurement_coordinator
        if coordinator is None:
            coordinator = PreviewMeasurementCoordinator()
            self._preview_measurement_coordinator = coordinator
        if (
            self._preview_measurement_timer is None
            and QtCore.QCoreApplication.instance() is not None
        ):
            parent = self.panel if isinstance(self.panel, QtCore.QObject) else None
            timer = QtCore.QTimer(parent)
            timer.setInterval(20)
            timer.timeout.connect(self._poll_preview_measurements)
            timer.start()
            self._preview_measurement_timer = timer
        return coordinator

    def _poll_preview_measurements(self) -> None:
        coordinator = self._preview_measurement_coordinator
        if coordinator is None:
            return
        result = coordinator.poll_latest()
        if result is None:
            return
        self._preview_measurement_pending.discard(result.key)
        self._preview_measurement_results[result.key] = result
        while len(self._preview_measurement_results) > 8:
            self._preview_measurement_results.pop(
                next(iter(self._preview_measurement_results))
            )
        self.panel._preview_measurement_metrics = coordinator.metrics()
        self.panel._latest_preview_measurement_result = result
        if result.attempt is not None:
            self.panel._latest_preview_pair_identity = (
                str(
                    result.attempt.left_detection.timestamp_metadata.get(
                        "frame_key",
                        result.attempt.left_detection.app_receive_monotonic_s,
                    )
                ),
                str(
                    result.attempt.right_detection.timestamp_metadata.get(
                        "frame_key",
                        result.attempt.right_detection.app_receive_monotonic_s,
                    )
                ),
            )
        self.panel._refresh_distance()
        self.panel._refresh_optical_alignment_view()

    def _preview_optical_payload(self, pair: object) -> dict[str, object]:
        expected_dy_px, _expected_source, _expected_level = (
            self.panel._optical_alignment_expected_dy_px()
        )
        mean_fy_px, _fy_source = self.panel._optical_alignment_mean_focal_px("y")
        mean_fx_px, _fx_source = self.panel._optical_alignment_mean_focal_px("x")
        timing = self._pair_timing_decision(pair)
        return {
            "left_center": tuple(pair.left.center),
            "right_center": tuple(pair.right.center),
            "expected_dy_px": expected_dy_px,
            "mean_fy_px": mean_fy_px,
            "mean_fx_px": mean_fx_px,
            "target_distance_m": self.panel.optical_target_distance_spin.value(),
            "left_pair_timestamp_s": timing.left_timestamp_s,
            "right_pair_timestamp_s": timing.right_timestamp_s,
        }

    def _preview_measurement_result(
        self,
        attempt: MeasurementAttempt,
        pair: object,
    ) -> PreviewMeasurementResult | None:
        key = attempt.canonical_sha256()
        cached = self._preview_measurement_results.get(key)
        if cached is not None:
            return cached
        if key not in self._preview_measurement_pending:
            self._preview_measurement_pending.add(key)
            self._ensure_preview_measurement_coordinator().submit(
                key=key,
                attempt=attempt,
                optical_payload=self._preview_optical_payload(pair),
            )
        return None

    def _update_detection(self, camera: str, detection: DetectionObservation | None) -> None:
        if detection is not None:
            decorate = getattr(self.panel, "_decorate_clock_epoch_metadata", None)
            if callable(decorate):
                detection = replace(
                    detection,
                    timestamp_metadata=decorate(
                        camera,
                        dict(detection.timestamp_metadata),
                    ),
                )
        if camera == "left":
            self.panel.latest_left = detection
        elif camera == "right":
            self.panel.latest_right = detection
        if detection is not None:
            self.panel.detection_buffers[camera].add(detection)
        coordinator = self._ensure_detection_pair_coordinator()
        reservation_source = None
        if detection is not None and self._exact_pair_reservation_supported():
            try:
                frame_key = FrameKey.from_mapping(detection.timestamp_metadata)
            except (TypeError, ValueError):
                frame_key = None
            pane = getattr(self.panel, f"{camera}_pane", None)
            resolver = getattr(pane, "native_ring_for_frame_key", None)
            if frame_key is not None and callable(resolver):
                reservation_source = resolver(frame_key)
        coordinator.submit(
            camera,
            detection,
            self._pairing_policy_snapshot(),
            reservation_source=reservation_source,
        )

    def _checkerboard_translation_right_in_left_m(self) -> tuple[float, float, float] | None:
        calibration = self.panel.active_intrinsics
        extrinsics = None if calibration is None else calibration.extrinsics
        if extrinsics is None:
            return None
        _rotation_right_to_left, translation_right_in_left_m = invert_left_to_right_camera_transform(
            extrinsics.rotation_left_to_right,
            extrinsics.translation_left_to_right_m,
        )
        return translation_right_in_left_m

    def _active_baseline_m(self) -> tuple[float, str, tuple[str, ...]]:
        if self.panel._distance_mode() == "checkerboard":
            translation = self.panel._checkerboard_translation_right_in_left_m()
            if translation is None:
                return 0.0, "checkerboard-waiting", ("waiting for checkerboard stereo extrinsics",)
            baseline_m = math.sqrt(sum(value * value for value in translation))
            if baseline_m <= 0.0:
                return 0.0, "checkerboard-waiting", ("checkerboard baseline length <= 0",)
            return baseline_m, "checkerboard", ()
        baseline = self.panel.latest_rtk_baseline
        if baseline is None:
            return 0.0, "gps-rtk-waiting", ("waiting for live GPS baseline",)
        usable, reason = self.panel._gps_baseline_usable_for_translation(baseline)
        if not usable:
            return baseline.baseline_m, "gps-rtk-blocked", (*baseline.warnings, reason)
        return baseline.baseline_m, "gps-rtk", baseline.warnings

    def _gps_fix_ready_for_baseline(self, state: GpsPositionState | None) -> bool:
        defaults = ReadinessConfig()
        manager = getattr(self.panel, "gnss_operation_manager", None)
        config = getattr(manager, "config", None)
        if config is None:
            config = getattr(getattr(self.panel, "settings", None), "gnss_operation", None)
        readiness = getattr(config, "readiness", None)
        minimum_fix_type = int(
            getattr(readiness, "minimum_fix_type", defaults.minimum_fix_type)
        )
        return (
            state is not None
            and state.has_position
            and state.fix_type is not None
            and state.fix_type >= minimum_fix_type
        )

    def _gps_pair_readiness(
        self,
        left: GpsPositionState | None,
        right: GpsPositionState | None,
    ) -> GpsPairReadiness:
        defaults = ReadinessConfig()
        manager = getattr(self.panel, "gnss_operation_manager", None)
        config = getattr(manager, "config", None)
        if config is None:
            config = getattr(getattr(self.panel, "settings", None), "gnss_operation", None)
        readiness = getattr(config, "readiness", None)
        return validate_gps_pair_for_baseline(
            left,
            right,
            now_s=time.monotonic(),
            maximum_age_s=float(
                getattr(
                    readiness,
                    "maximum_gps_sample_age_s",
                    defaults.maximum_gps_sample_age_s,
                )
            ),
            maximum_pair_delta_s=float(
                getattr(
                    readiness,
                    "maximum_left_right_timestamp_delta_ms",
                    defaults.maximum_left_right_timestamp_delta_ms,
                )
            )
            * 0.001,
            minimum_fix_type=int(
                getattr(readiness, "minimum_fix_type", defaults.minimum_fix_type)
            ),
        )

    def _gps_baseline_usable_for_translation(self, baseline: RtkBaselineEstimate | None) -> tuple[bool, str]:
        if baseline is None:
            return False, "live GPS baseline waiting"
        selector = getattr(self.panel, "_selected_baseline_gps_states", None)
        if not callable(selector):
            return False, "live GPS pair source unavailable"
        left, right = selector()
        readiness = self._gps_pair_readiness(left, right)
        if not readiness.usable:
            return False, "; ".join(readiness.reasons)
        if baseline.baseline_m <= 0.0:
            return False, "GPS baseline length <= 0"
        return True, "usable"

    def _rtk_fixed_status_text(self, baseline: RtkBaselineEstimate | None) -> str:
        selector = getattr(self.panel, "_selected_baseline_gps_states", None)
        if callable(selector):
            left, right = selector()
        else:
            left = self.panel.latest_gps.get("left")
            right = self.panel.latest_gps.get("right")
        source_reader = getattr(self.panel, "_pixhawk_gnss_source", None)
        left_source = str(source_reader("left")) if callable(source_reader) else "GPS1"
        right_source = str(source_reader("right")) if callable(source_reader) else "GPS1"

        def side_text(label: str, state: GpsPositionState | None) -> str:
            if state is None:
                return f"{label}=대기"
            sats = "--" if state.satellites_used is None else str(state.satellites_used)
            eph = "--" if state.eph_m is None else f"{state.eph_m:.2f}m"
            return f"{label}={state.rtk_label}(fix={state.fix_type}, sats={sats}, eph={eph})"

        fixed = left is not None and right is not None and left.fix_type == 6 and right.fix_type == 6
        usable, reason = self.panel._gps_baseline_usable_for_translation(baseline)
        fixed_text = "YES" if fixed else "NO"
        usability = "사용 가능" if usable else f"대기/차단: {reason}"
        return (
            f"RTK Fixed: {fixed_text} | GPS baseline 필수, {usability} | "
            f"{side_text(f'P1 {left_source}', left)} | {side_text(f'P2 {right_source}', right)}"
        )

    def _refresh_rtk_baseline(self) -> RtkBaselineEstimate | None:
        left_gps, right_gps = self.panel._selected_baseline_gps_states()
        baseline = estimate_rtk_baseline(
            left_gps,
            right_gps,
            now_s=time.monotonic(),
            require_fixed=False,
        )
        self.panel.latest_rtk_baseline = baseline
        self.panel._refresh_p1_body_frd_baseline()
        reference_text = " | 거리 계산 필수 입력"
        if hasattr(self.panel, "rtk_fixed_status_label"):
            self.panel.rtk_fixed_status_label.setText(self.panel._rtk_fixed_status_text(baseline))
        if baseline is None:
            self.panel._set_rtk_summary_chip("baseline", "Baseline: 대기 중", "idle")
            self.panel._set_rtk_summary_chip("camera", "Camera: 자세 대기 중", "idle")
            self.panel._set_rtk_summary_chip("status", "Status: GPS 대기 중", "idle")
            self.panel.rtk_baseline_label.setText(f"GPS baseline: 좌/우 GPS 대기 중{reference_text}")
            self.panel._refresh_rtk_alignment_view()
            self.panel._refresh_operation_header()
            self.panel._refresh_alignment_workflow_status()
            return None
        warning_text = f" | {'; '.join(baseline.warnings)}" if baseline.warnings else ""
        usable, reason = self.panel._gps_baseline_usable_for_translation(baseline)
        baseline_level = "good" if baseline.fix_quality == "GOOD" else "warn" if baseline.fix_quality == "DEGRADED" else "bad"
        status_level = "good" if usable else "bad"
        self.panel._set_rtk_summary_chip("baseline", f"Baseline {baseline.baseline_m:.3f} m", baseline_level)
        self.panel._set_rtk_summary_chip(
            "status",
            "Status: 사용 가능" if usable else f"Status: {reason}",
            status_level,
        )
        self.panel.rtk_baseline_label.setText(
            f"GPS baseline: {baseline.baseline_m:.3f} m "
            f"N/E/D={baseline.north_m:.3f}/{baseline.east_m:.3f}/{baseline.down_m:.3f} m "
            + (
                "P1 FRD=대기 중 "
                if self.panel.latest_p1_body_frd_baseline is None
                else (
                    "P1 FRD F/R/D="
                    f"{self.panel.latest_p1_body_frd_baseline.forward_m:.3f}/"
                    f"{self.panel.latest_p1_body_frd_baseline.right_m:.3f}/"
                    f"{self.panel.latest_p1_body_frd_baseline.down_m:.3f} m "
                )
            )
            + f"age={baseline.left_age_s * 1000:.0f}/{baseline.right_age_s * 1000:.0f}ms "
            f"quality={baseline.fix_quality}{reference_text}{warning_text}"
        )
        self.panel._refresh_rtk_alignment_view()
        self.panel._refresh_operation_header()
        self.panel._refresh_alignment_workflow_status()
        return baseline

    def _set_rtk_summary_chip(self, key: str, text: str, level: str) -> None:
        labels = getattr(self.panel, "rtk_summary_labels", {})
        label = labels.get(key)
        if label is not None:
            self.panel._set_health_label(label, text, level)

    def _baseline_forward_geometry_from_camera_translation(
        self, camera_translation: tuple[float, float, float] | None
    ) -> tuple[float, float, float, float] | None:
        if camera_translation is None:
            return None
        right_m, down_m, forward_m = camera_translation
        length_m = math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)
        if length_m <= 1e-9:
            return None
        ratio = max(-1.0, min(1.0, forward_m / length_m))
        angle_deg = math.degrees(math.acos(ratio))
        effective_m = math.sqrt(right_m * right_m + down_m * down_m)
        return angle_deg, effective_m, length_m, forward_m

    def _baseline_camera_translation_in_side_rdf_m(
        self,
        baseline: RtkBaselineEstimate,
        side: str,
    ) -> tuple[float, float, float] | None:
        extrinsic = self.panel.latest_extrinsics.get(side)
        if extrinsic is None:
            return None
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        body_vector = world_ned_vector_to_body_frd(world_vector, extrinsic.q)
        camera_vector = body_frd_vector_to_camera(body_vector)
        scale_to_m = self.panel.baseline_spin.value() if hasattr(self.panel, "baseline_spin") else None
        if scale_to_m is None:
            return camera_vector
        length_m = math.sqrt(sum(component * component for component in camera_vector))
        if length_m <= 1e-9:
            return None
        scale = float(scale_to_m) / length_m
        return (camera_vector[0] * scale, camera_vector[1] * scale, camera_vector[2] * scale)

    def _set_baseline_forward_metric_labels(
        self,
        *,
        prefix: str,
        geometry: tuple[float, float, float, float] | None,
        missing_text: str = "자세 대기 중",
    ) -> None:
        if not hasattr(self.panel, "baseline_attitude_labels"):
            return
        angle_key = "rdf_forward_angle" if prefix == "left" else "right_rdf_forward_angle"
        effective_key = "effective_baseline" if prefix == "left" else "right_effective_baseline"
        forward_key = "forward_component" if prefix == "left" else "right_forward_component"
        if geometry is None:
            self.panel._set_health_label(self.panel.baseline_attitude_labels[angle_key], missing_text, "warn")
            self.panel._set_health_label(self.panel.baseline_attitude_labels[effective_key], missing_text, "warn")
            self.panel._set_health_label(self.panel.baseline_attitude_labels[forward_key], missing_text, "warn")
            return
        angle_deg, effective_m, length_m, forward_m = geometry
        self.panel._set_health_label(
            self.panel.baseline_attitude_labels[angle_key],
            f"{angle_deg:.2f} deg ({'P1' if prefix == 'left' else 'P2'} RDF +Z vs P1->P2 baseline)",
            self.panel._baseline_forward_angle_level(angle_deg),
        )
        self.panel._set_label_text_if_changed(
            self.panel.baseline_attitude_labels[effective_key],
            f"{effective_m:.4f} m / {length_m:.4f} m",
        )
        self.panel._set_label_text_if_changed(
            self.panel.baseline_attitude_labels[forward_key],
            f"{forward_m:+.4f} m",
        )

    def _baseline_forward_angle_level(self, angle_deg: float) -> str:
        if 80.0 <= angle_deg <= 100.0:
            return "good"
        if 70.0 <= angle_deg <= 110.0:
            return "warn"
        return "bad"

    def _baseline_forward_geometry_text(self, camera_translation: tuple[float, float, float] | None) -> str:
        geometry = self.panel._baseline_forward_geometry_from_camera_translation(camera_translation)
        if geometry is None:
            return "P1 RDF Forward 각도: 자세 대기 중"
        angle_deg, effective_m, length_m, forward_m = geometry
        return (
            f"P1 RDF Forward vs baseline angle={angle_deg:.2f}°, "
            f"effective={effective_m:.3f}/{length_m:.3f} m, forward={forward_m:+.3f} m"
        )

    def _refresh_rtk_alignment_view(self) -> None:
        self.panel._refresh_p1_body_frd_baseline()
        if not hasattr(self.panel, "rtk_alignment_plot"):
            return
        baseline = self.panel.latest_rtk_baseline
        if baseline is None:
            self.panel.rtk_alignment_plot.set_alignment(
                None,
                left_attitude=self.panel.latest_extrinsics.get("left"),
                right_attitude=self.panel.latest_extrinsics.get("right"),
            )
            self.panel.rtk_alignment_detail_label.setText(
                "정렬 성분: 좌/우 GPS 위치 대기 중. 안테나 baseline은 카메라 광축 수직 baseline과 다를 수 있습니다."
            )
            return
        camera_translation = self.panel._alignment_view_camera_translation_right_in_left_m(baseline)
        left_ext = self.panel.latest_extrinsics.get("left")
        right_ext = self.panel.latest_extrinsics.get("right")
        self.panel.rtk_alignment_plot.set_alignment(baseline, camera_translation, left_ext, right_ext)
        antenna_text = (
            f"안테나 |T|={baseline.baseline_m:.3f} m, "
            f"N/E/D={baseline.north_m:.3f}/{baseline.east_m:.3f}/{baseline.down_m:.3f} m"
        )
        if camera_translation is None:
            self.panel._set_rtk_summary_chip("camera", "Camera: 자세 대기 중", "warn")
            body_text = "P1 body FRD=자세 대기 중"
            if self.panel.latest_p1_body_frd_baseline is not None:
                projection = self.panel.latest_p1_body_frd_baseline
                body_text = (
                    "P1 body FRD F/R/D="
                    f"{projection.forward_m:+.3f}/{projection.right_m:+.3f}/{projection.down_m:+.3f} m"
                )
            self.panel.rtk_alignment_detail_label.setText(
                f"정렬 성분: {antenna_text}. GPS-only 모드: N/E/D 기준선을 표시합니다. "
                f"{body_text}. 카메라 기준 Right/Down/Forward 변환은 픽스호크 자세 수신 후 추가됩니다."
            )
            return
        attitude_text = ""
        if left_ext is not None and right_ext is not None:
            relative = relative_attitude(left_ext.q, right_ext.q)
            attitude_text = (
                f" 상대 자세 ΔR/P/Y={relative.roll_deg:+.2f}/"
                f"{relative.pitch_deg:+.2f}/{relative.yaw_deg:+.2f}°."
            )
        x_m, y_m, z_m = camera_translation
        right_camera_translation = self.panel._baseline_camera_translation_in_side_rdf_m(baseline, "right")
        left_geometry = self.panel._baseline_forward_geometry_from_camera_translation(camera_translation)
        right_geometry = self.panel._baseline_forward_geometry_from_camera_translation(right_camera_translation)
        self.panel._set_baseline_forward_metric_labels(prefix="left", geometry=left_geometry)
        self.panel._set_baseline_forward_metric_labels(prefix="right", geometry=right_geometry)
        right_geometry_text = self.panel._baseline_forward_geometry_text(right_camera_translation).replace(
            "P1 RDF Forward", "P2 RDF Forward"
        )
        forward_warn = abs(z_m) > max(0.2, abs(x_m) * 0.5)
        down_warn = abs(y_m) > max(0.1, abs(x_m) * 0.25)
        self.panel._set_rtk_summary_chip(
            "camera",
            f"P1 RDF R/D/F {x_m:+.3f}/{y_m:+.3f}/{z_m:+.3f} m",
            "warn" if forward_warn or down_warn else "good",
        )
        self.panel.rtk_alignment_detail_label.setText(
            f"정렬 성분: {antenna_text}. "
            + (
                "P1 body FRD 대기 중. "
                if self.panel.latest_p1_body_frd_baseline is None
                else (
                    "P1 body FRD F/R/D="
                    f"{self.panel.latest_p1_body_frd_baseline.forward_m:+.3f}/"
                    f"{self.panel.latest_p1_body_frd_baseline.right_m:+.3f}/"
                    f"{self.panel.latest_p1_body_frd_baseline.down_m:+.3f} m. "
                )
            )
            + f"P1 RDF Right={x_m:.3f} m, Down={y_m:.3f} m, Forward={z_m:.3f} m. "
            f"P1 RDF 좌우 성분 |Right|={abs(x_m):.3f} m입니다. "
            f"{self.panel._baseline_forward_geometry_text(camera_translation)} | {right_geometry_text}.{attitude_text}"
        )

    def _optical_alignment_intrinsics_result(self) -> tuple[StereoIntrinsicsResult | None, str]:
        if self.panel.active_intrinsics is not None:
            return self.panel.active_intrinsics, "active calibration result"
        if self.panel.last_calibration_result is not None:
            return self.panel.last_calibration_result, "latest calibration result, not applied"
        return None, "intrinsics waiting"

    def _optical_alignment_mean_focal_px(self, axis: str) -> tuple[float | None, str]:
        result, source = self.panel._optical_alignment_intrinsics_result()
        if result is None:
            return None, source
        try:
            if axis == "x":
                left_value = float(result.left.camera_matrix[0][0])
                right_value = float(result.right.camera_matrix[0][0])
            else:
                left_value = float(result.left.camera_matrix[1][1])
                right_value = float(result.right.camera_matrix[1][1])
        except (TypeError, ValueError, IndexError):
            return None, "intrinsics focal invalid"
        mean_value = (left_value + right_value) / 2.0
        if not isfinite(mean_value) or mean_value <= 0.0:
            return None, "intrinsics focal invalid"
        return mean_value, source

    def _optical_alignment_expected_dy_px(self) -> tuple[float | None, str, str]:
        mode = str(self.panel.optical_expected_dy_combo.currentData() or "rectified")
        if mode == "rectified":
            return 0.0, "Rectified 0 px", "good"
        if mode == "manual":
            return self.panel.optical_expected_dy_spin.value(), "Manual", "good"
        result, source = self.panel._optical_alignment_intrinsics_result()
        if result is None:
            return None, source, "warn"
        try:
            left_cy = float(result.left.camera_matrix[1][2])
            right_cy = float(result.right.camera_matrix[1][2])
        except (TypeError, ValueError, IndexError):
            return None, "intrinsics cy invalid", "bad"
        expected = left_cy - right_cy
        return expected, f"Raw intrinsics cy diff ({source})", "warn" if result is not self.panel.active_intrinsics else "good"

    def _optical_alignment_level_from_mrad(self, value_mrad: float | None) -> str:
        if value_mrad is None or not isfinite(value_mrad):
            return "idle"
        magnitude = abs(value_mrad)
        if magnitude <= 0.2:
            return "good"
        if magnitude <= 0.6:
            return "warn"
        return "bad"

    def _optical_alignment_overall_level(self, levels: Sequence[str]) -> str:
        if any(level == "bad" for level in levels):
            return "bad"
        if any(level == "warn" for level in levels):
            return "warn"
        if any(level == "good" for level in levels):
            return "good"
        return "idle"

    def _optical_alignment_level_text(self, level: str) -> str:
        return {"good": "GOOD", "warn": "WARN", "bad": "BAD", "idle": "WAITING"}.get(level, "WAITING")

    def _format_optical_px(self, value: float | None) -> str:
        return "--" if value is None or not isfinite(value) else f"{value:+.3f} px"

    def _format_optical_mrad(self, value: float | None) -> str:
        return "--" if value is None or not isfinite(value) else f"{value:+.3f} mrad"

    def _refresh_optical_alignment_view(self) -> None:
        if not hasattr(self.panel, "optical_alignment_labels"):
            return
        labels = self.panel.optical_alignment_labels
        pair = self.panel._matched_detection_pair()
        if pair is None:
            self.panel._set_health_label(
                self.panel.optical_alignment_status_label,
                "Optical alignment: YOLO target pair 대기 중",
                "idle",
            )
            for label in labels.values():
                self.panel._set_health_label(label, "대기 중", "idle")
            expected_dy, expected_source, expected_level = self.panel._optical_alignment_expected_dy_px()
            self.panel._set_health_label(
                labels["expected_dy"],
                f"{self.panel._format_optical_px(expected_dy)} | {expected_source}",
                expected_level,
            )
            return
        left_center_x, left_center_y = pair.left.center
        right_center_x, right_center_y = pair.right.center
        expected_dy_px, expected_dy_source, expected_dy_level = self.panel._optical_alignment_expected_dy_px()
        mean_fy_px, fy_source = self.panel._optical_alignment_mean_focal_px("y")
        mean_fx_px, fx_source = self.panel._optical_alignment_mean_focal_px("x")
        target_distance_m = self.panel.optical_target_distance_spin.value()
        current_identity = (
            str(pair.left.timestamp_metadata.get("frame_key") or pair.left.timestamp_s),
            str(pair.right.timestamp_metadata.get("frame_key") or pair.right.timestamp_s),
        )
        preview_result = getattr(
            self.panel,
            "_latest_preview_measurement_result",
            None,
        )
        if (
            not isinstance(preview_result, PreviewMeasurementResult)
            or getattr(self.panel, "_latest_preview_pair_identity", None)
            != current_identity
        ):
            try:
                optical_attempt = self.panel._build_measurement_attempt(pair)
            except (TypeError, ValueError):
                optical_attempt = None
            if optical_attempt is not None:
                self._preview_measurement_result(optical_attempt, pair)
            self.panel._set_health_label(
                self.panel.optical_alignment_status_label,
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
        vertical_level = self.panel._optical_alignment_level_from_mrad(vertical_error_mrad)
        attempt = preview_result.attempt
        geometry_result = preview_result.geometry_result
        geometry_error = preview_result.error
        baseline_m = 0.0 if attempt is None else attempt.baseline_m
        baseline_source = "waiting" if attempt is None else attempt.baseline_source
        yaw_level = self.panel._optical_alignment_level_from_mrad(yaw_error_mrad)
        overall_level = self.panel._optical_alignment_overall_level((vertical_level, yaw_level))

        class_text = pair.left.class_name or str(pair.left.class_id)
        confidence_text = f"L={pair.left.confidence:.2f}, R={pair.right.confidence:.2f}"
        self.panel._set_health_label(
            labels["left_center"],
            f"x={left_center_x:.1f}, y={left_center_y:.1f} | {pair.left.width}x{pair.left.height}",
            "good",
        )
        self.panel._set_health_label(
            labels["right_center"],
            f"x={right_center_x:.1f}, y={right_center_y:.1f} | {pair.right.width}x{pair.right.height}",
            "good",
        )
        self.panel._set_health_label(
            labels["bbox_delta"],
            f"dx={observed_dx_px:+.3f} px, dy={observed_dy_px:+.3f} px",
            "good",
        )
        self.panel._set_health_label(labels["bbox_quality"], f"class={class_text} | conf {confidence_text}", "good")
        camera_delta_text = "--" if camera_delta_s is None else f"{camera_delta_s * 1000.0:.1f} ms"
        self.panel._set_health_label(labels["camera_time"], camera_delta_text, "good" if camera_delta_s is not None else "warn")

        self.panel._set_health_label(
            labels["expected_dy"],
            f"{self.panel._format_optical_px(expected_dy_px)} | {expected_dy_source}",
            expected_dy_level,
        )
        self.panel._set_health_label(labels["observed_dy"], self.panel._format_optical_px(observed_dy_px), "good")
        self.panel._set_health_label(labels["dy_residual"], self.panel._format_optical_px(dy_residual_px), vertical_level)
        self.panel._set_health_label(
            labels["vertical_error"],
            f"{self.panel._format_optical_mrad(vertical_error_mrad)} | fy={mean_fy_px:.1f}px"
            if mean_fy_px is not None
            else f"-- | {fy_source}",
            vertical_level,
        )

        self.panel._set_health_label(labels["observed_dx"], self.panel._format_optical_px(observed_dx_px), "good")
        self.panel._set_health_label(
            labels["expected_disparity"],
            f"{self.panel._format_optical_px(expected_disparity_px)} | target={target_distance_m:.2f} m",
            "good" if expected_disparity_px is not None else "warn",
        )
        self.panel._set_health_label(labels["yaw_residual"], self.panel._format_optical_px(yaw_residual_px), yaw_level)
        self.panel._set_health_label(
            labels["yaw_error"],
            f"{self.panel._format_optical_mrad(yaw_error_mrad)} | fx={mean_fx_px:.1f}px"
            if mean_fx_px is not None
            else f"-- | {fx_source}",
            yaw_level,
        )

        triangulated = None if geometry_result is None else geometry_result.triangulated
        estimate = None if geometry_result is None else geometry_result.estimate
        if triangulated is None:
            reason = geometry_error or ("" if geometry_result is None else geometry_result.triangulation_reason)
            self.panel._set_health_label(labels["range"], "대기 중", "idle")
            self.panel._set_health_label(labels["ray_gap"], reason or "geometry 대기 중", "warn")
        else:
            self.panel._set_health_label(labels["range"], f"{triangulated.range_m:.3f} m", "good")
            self.panel._set_health_label(labels["ray_gap"], f"{triangulated.closest_gap_m:.3f} m", "good")
        self.panel._set_health_label(labels["baseline"], f"{baseline_source}:{baseline_m:.3f} m", "good" if baseline_m > 0.0 else "warn")
        triangulation_text = (
            "대기 중"
            if attempt is None
            else f"{attempt.triangulation_source} | {geometry_result.triangulation_reason if geometry_result else geometry_error}"
        )
        self.panel._set_health_label(labels["triangulation"], triangulation_text, "good" if triangulated is not None else "warn")
        attitude_status = "WAITING"
        if attempt is not None:
            uses_live_pixhawk = "live_pixhawk" in dict(
                attempt.measurement_component_sources
            ).values()
            attitude_status = classify_attitude_timing(
                uses_pixhawk_attitude=uses_live_pixhawk,
                left_age_s=None if estimate is None else estimate.left_attitude_age_s,
                right_age_s=None if estimate is None else estimate.right_attitude_age_s,
                pair_delta_s=attempt.attitude_pair_delta_s,
            )
        self.panel._set_health_label(
            labels["attitude"],
            attitude_status,
            self.panel._attitude_timing_health_level(attitude_status),
        )

        self.panel._set_health_label(
            self.panel.optical_alignment_status_label,
            (
                f"Optical alignment: {self.panel._optical_alignment_level_text(overall_level)} | "
                f"vertical={self.panel._format_optical_mrad(vertical_error_mrad)} | "
                f"yaw={self.panel._format_optical_mrad(yaw_error_mrad)} | "
                f"mode={self.panel._distance_mode_label()}"
            ),
            overall_level,
        )

    def _gnss_readiness_config(self) -> object:
        manager = getattr(self.panel, "gnss_operation_manager", None)
        config = getattr(manager, "config", None)
        if config is None:
            config = getattr(getattr(self.panel, "settings", None), "gnss_operation", None)
        return getattr(config, "readiness", None)

    def _measurement_correction_frame_token(self) -> str:
        """Return the identity of the frame the retained rover positions resolve in.

        P1 and P2 can only be connected one at a time, so their retained
        positions are captured minutes apart.  Two RTK fixes are comparable as
        one P1->P2 baseline only while the correction source and the Base
        coordinate they resolve against stay the same; a re-survey, a
        mountpoint/caster change, or an operation-mode change silently moves
        the frame.  Binding the retained pair to this token drops it instead.
        """

        manager = getattr(self.panel, "gnss_operation_manager", None)
        identity = getattr(manager, "source_identity", None)
        source_id = "" if identity is None else str(getattr(identity, "stable_id", "") or "")
        profile = getattr(self.panel, "gnss_fixed_base_profile", None)
        base_hash = "" if profile is None else str(getattr(profile, "profile_hash", "") or "")
        # A VRS mountpoint keeps its caster/mountpoint identity while placing a
        # new virtual station per session, so the configured identity alone
        # cannot see the frame move.  The observed RTCM 1005/1006 station does.
        reference = getattr(self.panel, "latest_rtcm_reference_station", None)
        station = ""
        if isinstance(reference, Mapping):
            ecef = reference.get("ecef_01mm")
            try:
                coordinates = ",".join(str(int(value)) for value in ecef or ())
            except (TypeError, ValueError):
                coordinates = ""
            if coordinates:
                station = f"{reference.get('station_id')}:{coordinates}"
        return f"{source_id}\0{base_hash}\0{station}"

    @staticmethod
    def _attitude_state_is_valid_for_measurement_cache(state: object) -> bool:
        if state is None:
            return False
        try:
            timestamp_s = float(getattr(state, "timestamp_s", float("nan")))
        except (TypeError, ValueError):
            return False
        if not math.isfinite(timestamp_s):
            return False
        try:
            quaternion = tuple(float(value) for value in getattr(state, "q", ()) or ())
        except (TypeError, ValueError):
            return False
        if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
            return False
        return any(value != 0.0 for value in quaternion)

    def _cached_measurement_attitude_states(
        self,
    ) -> tuple[object | None, object | None, tuple[str, ...]]:
        """Update each P1/P2 side independently and retain last valid attitudes.

        This is the attitude twin of `_cached_measurement_gps_states`.  Both
        sensor halves of the live Pixhawk method therefore reuse their own last
        valid process-local sample instead of one half being retained while the
        other is re-read live and blocks on freshness.
        """

        panel = self.panel
        cache = getattr(panel, "_measurement_latest_valid_attitude", None)
        if not isinstance(cache, dict):
            cache = {"left": None, "right": None}
            panel._measurement_latest_valid_attitude = cache
        live = getattr(panel, "latest_extrinsics", {}) or {}
        measurement_start = getattr(panel, "_measurement_started_monotonic_s", None)
        reused: list[str] = []
        for side in ("left", "right"):
            sample = live.get(side) if isinstance(live, Mapping) else None
            if self._attitude_state_is_valid_for_measurement_cache(sample):
                previous = cache.get(side)
                previous_timestamp = float(getattr(previous, "timestamp_s", -1.0) or -1.0)
                sample_timestamp = float(getattr(sample, "timestamp_s", -1.0) or -1.0)
                if previous is None or sample_timestamp >= previous_timestamp:
                    cache[side] = sample
            selected = cache.get(side)
            selected_timestamp = getattr(selected, "timestamp_s", None)
            reused_from_before_measurement = bool(
                selected is not None
                and measurement_start is not None
                and selected_timestamp is not None
                and float(selected_timestamp) < float(measurement_start)
            )
            if selected is not None and (
                selected is not sample or reused_from_before_measurement
            ):
                reused.append(f"{side}_attitude")
        return cache.get("left"), cache.get("right"), tuple(reused)

    def _gps_state_is_valid_for_measurement_cache(self, state: object) -> bool:
        defaults = ReadinessConfig()
        readiness = self._gnss_readiness_config()
        minimum_fix_type = int(
            getattr(readiness, "minimum_fix_type", defaults.minimum_fix_type)
        )
        if state is None or int(getattr(state, "fix_type", 0) or 0) < minimum_fix_type:
            return False
        values = (
            getattr(state, "latitude_deg", None),
            getattr(state, "longitude_deg", None),
            getattr(state, "altitude_m", None),
        )
        try:
            return all(value is not None and math.isfinite(float(value)) for value in values)
        except (TypeError, ValueError):
            return False

    def _cached_measurement_gps_states(
        self,
    ) -> tuple[object | None, object | None, tuple[str, ...]]:
        """Update each P1/P2 side independently and retain last valid 3D fixes."""

        panel = self.panel
        cache = getattr(panel, "_measurement_latest_valid_gps", None)
        if not isinstance(cache, dict):
            cache = {"left": None, "right": None}
            panel._measurement_latest_valid_gps = cache
        source_cache = getattr(panel, "_measurement_latest_valid_gps_source", None)
        if not isinstance(source_cache, dict):
            source_cache = {}
            panel._measurement_latest_valid_gps_source = source_cache
        frame_cache = getattr(panel, "_measurement_latest_valid_gps_correction_frame", None)
        if not isinstance(frame_cache, dict):
            frame_cache = {}
            panel._measurement_latest_valid_gps_correction_frame = frame_cache
        frame_barrier = getattr(panel, "_measurement_gps_correction_frame_barrier_s", None)
        if not isinstance(frame_barrier, dict):
            frame_barrier = {}
            panel._measurement_gps_correction_frame_barrier_s = frame_barrier
        correction_frame = self._measurement_correction_frame_token()
        current_left, current_right = panel._selected_baseline_gps_states()
        current = {"left": current_left, "right": current_right}
        measurement_start = getattr(panel, "_measurement_started_monotonic_s", None)
        reused: list[str] = []
        for side in ("left", "right"):
            source_reader = getattr(panel, "_pixhawk_gnss_source", None)
            source = (
                str(source_reader(side))
                if callable(source_reader)
                else ("GPS2" if str(getattr(current[side], "topic", "")).endswith("GPS2_RAW") else "GPS1")
            )
            if source_cache.get(side) != source:
                cache[side] = None
                source_cache[side] = source
            previous_frame = frame_cache.get(side)
            if previous_frame != correction_frame:
                frame_cache[side] = correction_frame
                if previous_frame is not None:
                    # The frame moved under a side that is currently unplugged.
                    # Its last received sample still resolves against the old
                    # Base coordinate, so it is dropped and only a sample
                    # observed after the change may be retained again.
                    cache[side] = None
                    frame_barrier[side] = time.monotonic()
            sample = current[side]
            barrier_s = frame_barrier.get(side)
            admissible = self._gps_state_is_valid_for_measurement_cache(sample)
            if admissible and barrier_s is not None:
                try:
                    admissible = float(sample.timestamp_s) >= float(barrier_s)
                except (AttributeError, TypeError, ValueError):
                    admissible = False
            if admissible:
                previous = cache.get(side)
                previous_timestamp = float(getattr(previous, "timestamp_s", -1.0) or -1.0)
                sample_timestamp = float(getattr(sample, "timestamp_s", -1.0) or -1.0)
                if previous is None or sample_timestamp >= previous_timestamp:
                    cache[side] = sample
            selected = cache.get(side)
            selected_timestamp = getattr(selected, "timestamp_s", None)
            reused_from_before_measurement = bool(
                selected is not None
                and measurement_start is not None
                and selected_timestamp is not None
                and float(selected_timestamp) < float(measurement_start)
            )
            if selected is not None and (
                selected is not sample or reused_from_before_measurement
            ):
                reused.append(f"{side}_gps")
        # A side left empty by a frame change is otherwise indistinguishable
        # from one that never delivered, which is the difference an operator
        # needs to act on: reconnect that board to refix under the new
        # reference rather than hunt for a missing device.
        panel._measurement_gps_correction_frame_dropped = tuple(
            side
            for side in ("left", "right")
            if cache.get(side) is None and frame_barrier.get(side) is not None
        )
        return cache.get("left"), cache.get("right"), tuple(reused)

    @staticmethod
    def _ftp_command_round_sync(pair: object) -> dict[str, object]:
        left_metadata = getattr(getattr(pair, "left", None), "timestamp_metadata", {}) or {}
        right_metadata = getattr(getattr(pair, "right", None), "timestamp_metadata", {}) or {}
        if not isinstance(left_metadata, Mapping) or not isinstance(right_metadata, Mapping):
            return {}
        left_capture = str(left_metadata.get("capture_id") or "")
        right_capture = str(right_metadata.get("capture_id") or "")
        left_round = str(
            left_metadata.get("command_round")
            or left_metadata.get("capture_round")
            or ""
        )
        right_round = str(
            right_metadata.get("command_round")
            or right_metadata.get("capture_round")
            or ""
        )
        def capture_ready(metadata: Mapping[str, object]) -> bool:
            downloaded = bool(
                metadata.get("ftp_downloaded", metadata.get("downloaded", False))
            )
            return bool(
                metadata.get(
                    "capture_command_sent",
                    metadata.get("command_sent", True),
                )
                and metadata.get("stored", downloaded)
                and downloaded
                and metadata.get("decoded", False)
            )

        left_ready = capture_ready(left_metadata)
        right_ready = capture_ready(right_metadata)
        if not (
            left_capture
            and left_capture == right_capture
            and left_round
            and left_round == right_round
            and left_ready
            and right_ready
        ):
            return {}
        return {
            "capture_id": left_capture,
            "command_round": left_round,
            "time_basis": "command_round_assumed_sync",
            "pair_delta_s": None,
            "operator_approved_sync_assumption": True,
            "warning": (
                "FTP command-round simultaneity is operator-approved; "
                "it is not a measured exposure timestamp"
            ),
        }

    def _pair_timing_decision(self, pair: object) -> PairTimingDecision:
        result = getattr(self.panel, "_coordinated_pairing_result", None)
        if result is not None and getattr(result, "pair", None) is pair:
            timing = getattr(result, "timing", None)
            if isinstance(timing, PairTimingDecision):
                return timing
        if bool(getattr(self.panel, "_detection_pair_coordinator_active", False)):
            raise ValueError("matched detection pair has no coordinated timing decision")

        requested_basis = self.requested_time_basis()
        effective_basis = requested_basis
        fallback_reason = ""
        left_timestamp_s = detection_timestamp_for_basis(pair.left, requested_basis)
        right_timestamp_s = detection_timestamp_for_basis(pair.right, requested_basis)
        if (
            requested_basis == "camera_rtcp"
            and (left_timestamp_s is None or right_timestamp_s is None)
        ):
            effective_basis = TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC
            fallback_reason = "RTCP_TIMESTAMP_UNAVAILABLE"
            left_timestamp_s = detection_timestamp_for_basis(
                pair.left,
                effective_basis,
            )
            right_timestamp_s = detection_timestamp_for_basis(
                pair.right,
                effective_basis,
            )
        if left_timestamp_s is None or right_timestamp_s is None:
            raise ValueError("matched detection pair has no timestamp on its selected basis")
        if effective_basis == TIMESTAMP_BASIS_APP_RECEIVE_MONOTONIC:
            left_ns = pair.left.app_receive_monotonic_ns
            right_ns = pair.right.app_receive_monotonic_ns
            signed_delta_ns = right_ns - left_ns
            left_timestamp_s = left_ns * 1.0e-9
            right_timestamp_s = right_ns * 1.0e-9
        else:
            signed_delta_ns = int(
                round(
                    (float(right_timestamp_s) - float(left_timestamp_s))
                    * 1_000_000_000.0
                )
            )
        return PairTimingDecision(
            requested_basis=requested_basis,
            effective_basis=effective_basis,
            left_timestamp_s=float(left_timestamp_s),
            right_timestamp_s=float(right_timestamp_s),
            signed_delta_ns=signed_delta_ns,
            abs_delta_ns=abs(signed_delta_ns),
            fallback_reason=fallback_reason,
        )

    def _freeze_measurement_pair_inputs(self, pair: object) -> dict[str, object]:
        """Persist native pixels, then freeze one shared, byte-free pair snapshot."""

        panel = self.panel
        timing = self._pair_timing_decision(pair)
        requested_timestamp_basis = timing.requested_basis
        timestamp_basis = timing.effective_basis
        left_pairing_timestamp_s = timing.left_timestamp_s
        right_pairing_timestamp_s = timing.right_timestamp_s
        physical_key = detection_pair_key(
            triangulation_source="all_methods",
            timestamp_basis=timestamp_basis,
            left=pair.left,
            right=pair.right,
            left_pairing_timestamp_s=left_pairing_timestamp_s,
            right_pairing_timestamp_s=right_pairing_timestamp_s,
        )
        cached = getattr(panel, "_last_frozen_measurement_pair", None)
        if (
            isinstance(cached, Mapping)
            and tuple(cached.get("physical_key", ())) == tuple(physical_key)
        ):
            reservation = getattr(
                panel,
                "_coordinated_detection_pair_reservation",
                None,
            )
            self._release_detection_pair_reservation(reservation)
            panel._coordinated_detection_pair_reservation = None
            return dict(cached)

        created_monotonic_ns = time.monotonic_ns()
        created_unix_ns = time.time_ns()
        detection_midpoint_s = (
            float(pair.left.timestamp_s) + float(pair.right.timestamp_s)
        ) / 2.0
        left_detection = DetectionSnapshot.from_observation(
            pair.left,
            pairing_timestamp_basis=timestamp_basis,
            pairing_timestamp_s=left_pairing_timestamp_s,
        )
        right_detection = DetectionSnapshot.from_observation(
            pair.right,
            pairing_timestamp_basis=timestamp_basis,
            pairing_timestamp_s=right_pairing_timestamp_s,
        )
        left_nearest = panel.attitude_buffers["left"].nearest(float(pair.left.timestamp_s))
        right_nearest = panel.attitude_buffers["right"].nearest(float(pair.right.timestamp_s))
        measurement_start = getattr(panel, "_measurement_started_monotonic_s", None)
        (
            retained_left_attitude,
            retained_right_attitude,
            reused_attitude_inputs,
        ) = self._cached_measurement_attitude_states()
        left_attitude_state = (
            retained_left_attitude if left_nearest is None else left_nearest.sample
        )
        right_attitude_state = (
            retained_right_attitude if right_nearest is None else right_nearest.sample
        )
        left_attitude = (
            None
            if left_attitude_state is None
            else AttitudeSnapshot.from_state(left_attitude_state)
        )
        right_attitude = (
            None
            if right_attitude_state is None
            else AttitudeSnapshot.from_state(right_attitude_state)
        )
        left_attitude_age_s = (
            None
            if left_attitude is None
            else abs(float(pair.left.timestamp_s) - float(left_attitude.timestamp_s))
        )
        right_attitude_age_s = (
            None
            if right_attitude is None
            else abs(float(pair.right.timestamp_s) - float(right_attitude.timestamp_s))
        )
        selected_left_gps, selected_right_gps, reused_gps_inputs = (
            self._cached_measurement_gps_states()
        )
        reused_inputs = tuple(
            dict.fromkeys((*reused_gps_inputs, *reused_attitude_inputs))
        )
        left_gps = None if selected_left_gps is None else GpsSnapshot.from_state(selected_left_gps)
        right_gps = None if selected_right_gps is None else GpsSnapshot.from_state(selected_right_gps)
        pair_sequence = int(getattr(panel, "_measurement_attempt_counter", 0)) + 1
        panel._measurement_attempt_counter = pair_sequence
        generation_id = str(
            getattr(panel, "_measurement_attempt_generation_id", "")
            or getattr(panel, "_measurement_id", "")
            or "measurement"
        )
        measurement_id = str(getattr(panel, "_measurement_id", "") or generation_id)
        pair_id = f"{generation_id}:{pair_sequence:020d}"
        native_fields: dict[str, object] = {}
        persister = getattr(panel, "_persist_native_detection_pair", None)
        pair_reservation = getattr(
            panel,
            "_coordinated_detection_pair_reservation",
            None,
        )
        if not isinstance(pair_reservation, RingPairReservation):
            pair_reservation = None
        if callable(persister):
            try:
                native_fields = dict(
                    persister(
                        left_detection=left_detection,
                        right_detection=right_detection,
                        pair_id=f"pair-{pair_sequence}",
                        purpose="method-evaluation",
                        reservation=pair_reservation,
                    )
                    or {}
                )
            except Exception:
                self._release_detection_pair_reservation(pair_reservation)
                panel._coordinated_detection_pair_reservation = None
                raise
        if pair_reservation is not None and not pair_reservation.active:
            panel._coordinated_detection_pair_reservation = None
        if native_fields.get("native_pair_status") == "EVIDENCE_FRAME_UNAVAILABLE":
            raise ValueError(
                "EVIDENCE_FRAME_UNAVAILABLE: exact selected native frame is no longer available"
            )
        capture_sync = self._ftp_command_round_sync(pair)
        if not capture_sync:
            configured_delta_s = float(panel._yolo_pair_max_delta_s())
            timing_limit_ns = (
                100_000_000
                if not math.isfinite(configured_delta_s)
                else min(
                    100_000_000,
                    max(0, int(round(configured_delta_s * 1_000_000_000.0))),
                )
            )
            capture_sync = live_pair_timing_evidence(
                left_detection,
                right_detection,
                requested_basis=requested_timestamp_basis,
                effective_basis=timestamp_basis,
                fallback_reason=timing.fallback_reason,
                max_abs_delta_ns=timing_limit_ns,
            )
        snapshot = FrozenMeasurementPairSnapshot(
            measurement_id=measurement_id,
            pair_id=pair_id,
            pair_sequence=pair_sequence,
            created_monotonic_ns=created_monotonic_ns,
            created_unix_ns=created_unix_ns,
            timestamp_basis=timestamp_basis,
            left_detection=asdict(left_detection),
            right_detection=asdict(right_detection),
            left_attitude=None if left_attitude is None else asdict(left_attitude),
            right_attitude=None if right_attitude is None else asdict(right_attitude),
            left_gps=None if left_gps is None else asdict(left_gps),
            right_gps=None if right_gps is None else asdict(right_gps),
            native_frames=native_fields,
            capture_sync=capture_sync,
            reused_inputs=reused_inputs,
        )
        frozen = {
            "physical_key": tuple(physical_key),
            "snapshot": snapshot,
            "requested_timestamp_basis": requested_timestamp_basis,
            "timestamp_basis": timestamp_basis,
            "timing_fallback_reason": timing.fallback_reason,
            "pair_timing_decision": timing,
            "left_pairing_timestamp_s": float(left_pairing_timestamp_s),
            "right_pairing_timestamp_s": float(right_pairing_timestamp_s),
            "created_monotonic_s": created_monotonic_ns * 1.0e-9,
            "created_wall_s": created_unix_ns * 1.0e-9,
            "detection_midpoint_s": detection_midpoint_s,
            "left_detection": left_detection,
            "right_detection": right_detection,
            "left_attitude": left_attitude,
            "right_attitude": right_attitude,
            "left_attitude_age_s": left_attitude_age_s,
            "right_attitude_age_s": right_attitude_age_s,
            "left_gps": left_gps,
            "right_gps": right_gps,
            "measurement_start": measurement_start,
            "reused_inputs": reused_inputs,
        }
        panel._last_frozen_measurement_pair = frozen
        return dict(frozen)

    def _build_measurement_attempt(
        self,
        pair,
        *,
        method_status: MeasurementMethodStatus | None = None,
        frozen_inputs: Mapping[str, object] | None = None,
        use_cache: bool = True,
    ) -> MeasurementAttempt:
        """Resolve one method only from a previously frozen shared pair."""

        method_status = method_status or self._select_measurement_method_for_attempt()
        if method_status is None:
            raise ValueError("no ready measurement method")
        method_id = method_status.method_id
        pixhawk_geometry_method = method_id in {
            PIXHAWK_RTK_LIVE,
            PIXHAWK_RP_GPS_BASELINE_YAW,
        }
        baseline_yaw_method = method_id == PIXHAWK_RP_GPS_BASELINE_YAW
        triangulation_source = "pixhawk" if pixhawk_geometry_method else "checkerboard"
        geometry_kind = (
            "live_pixhawk_baseline_yaw"
            if baseline_yaw_method
            else "live_pixhawk_mount"
            if pixhawk_geometry_method
            else "relative_pose"
        )
        frozen = dict(frozen_inputs or self._freeze_measurement_pair_inputs(pair))
        snapshot = frozen.get("snapshot")
        if not isinstance(snapshot, FrozenMeasurementPairSnapshot):
            raise TypeError("frozen measurement pair snapshot missing")
        timestamp_basis = str(frozen["timestamp_basis"])
        left_pairing_timestamp_s = float(frozen["left_pairing_timestamp_s"])
        right_pairing_timestamp_s = float(frozen["right_pairing_timestamp_s"])
        created_monotonic_s = float(frozen["created_monotonic_s"])
        detection_midpoint_s = float(frozen["detection_midpoint_s"])
        left_detection = frozen["left_detection"]
        right_detection = frozen["right_detection"]
        left_attitude = frozen.get("left_attitude")
        right_attitude = frozen.get("right_attitude")
        left_attitude_age_s = frozen.get("left_attitude_age_s")
        right_attitude_age_s = frozen.get("right_attitude_age_s")
        attitude_pair_delta_s = (
            None
            if left_attitude is None or right_attitude is None
            else abs(left_attitude.timestamp_s - right_attitude.timestamp_s)
        )
        left_gps = frozen.get("left_gps")
        right_gps = frozen.get("right_gps")
        measurement_start = frozen.get("measurement_start")
        yaw_selection: BaselineYawSelection | None = None
        yaw_p1_age_s: float | None = None
        yaw_p2_age_s: float | None = None
        geometry_left_quaternion = None
        geometry_right_quaternion = None
        if baseline_yaw_method:
            yaw_selection, yaw_p1_age_s, yaw_p2_age_s = self._baseline_yaw_selection_preview(
                left_gps=left_gps,
                right_gps=right_gps,
                left_attitude=left_attitude,
                right_attitude=right_attitude,
            )
            if yaw_selection is None or not yaw_selection.ready:
                raise ValueError(
                    "baseline yaw selection unavailable: "
                    + ("no GPS baseline or Pixhawk yaw reference" if yaw_selection is None else yaw_selection.reason)
                )
            if left_attitude is None or right_attitude is None or yaw_selection.selected_yaw_deg is None:
                raise ValueError("baseline yaw method requires both Pixhawk attitudes")
            geometry_left_quaternion = quaternion_from_roll_pitch_yaw_deg(
                left_attitude.roll_deg,
                left_attitude.pitch_deg,
                yaw_selection.selected_yaw_deg,
            )
            geometry_right_quaternion = quaternion_from_roll_pitch_yaw_deg(
                right_attitude.roll_deg,
                right_attitude.pitch_deg,
                yaw_selection.selected_yaw_deg,
            )
        relative = (
            None
            if geometry_left_quaternion is None or geometry_right_quaternion is None
            else relative_attitude(geometry_left_quaternion, geometry_right_quaternion)
        )
        attitude_for_geometry_left = left_attitude
        attitude_for_geometry_right = right_attitude
        measurement_result = self._measurement_intrinsics_result_for_method(
            method_status,
            left_attitude=attitude_for_geometry_left,
            right_attitude=attitude_for_geometry_right,
            left_gps=left_gps,
            right_gps=right_gps,
        )
        intrinsics = None
        if measurement_result is not None:
            nominal_quality = (
                dict(measurement_result.calibration_quality or {}).get("state")
                == NOMINAL_INTRINSICS_QUALITY
            )
            frozen_kd = bool(
                getattr(
                    self.panel,
                    "_measurement_frozen_comparison_inputs_declared",
                    False,
                )
            )
            operating_profile = (
                None
                if frozen_kd
                else self.panel._optical_pose_controller()._operating_intrinsics_profile()
                if nominal_quality
                else None
            )
            profile_path = str(
                getattr(
                    self.panel,
                    "_pending_measurement_frozen_intrinsics_path",
                    "",
                )
                if frozen_kd
                else getattr(operating_profile, "source_path", "")
                or getattr(self.panel, "_active_intrinsics_profile_path", "")
                or f"memory:active-calibration:zoom-{self.panel.zoom_level_spin.value():02d}"
            )
            intrinsics = StereoIntrinsicsSnapshot.from_result(
                measurement_result,
                active_zoom_level=self.panel.zoom_level_spin.value(),
                selected_profile_path=profile_path,
                selected_extrinsics_profile_path=method_status.instance_id,
                intrinsics_source_zoom_level=getattr(
                    self.panel, "_active_intrinsics_source_zoom_level", None
                ) if operating_profile is None else operating_profile.left.source_zoom_index,
                extrinsics_source_zoom_level=getattr(
                    self.panel, "_active_extrinsics_source_zoom_level", None
                ),
                selection_mode=f"measurement_method/{method_id}",
            )

        warnings: tuple[str, ...] = tuple(
            dict.fromkeys(
                (*method_status.warnings, *tuple(getattr(self.panel, "_last_pair_advisories", ()) or ()))
            )
        )
        if baseline_yaw_method and yaw_selection is not None:
            warnings = tuple(
                dict.fromkeys(
                    (
                        *warnings,
                        "yaw assumed perpendicular to GPS baseline (+/-90deg)",
                        f"baseline yaw branch={yaw_selection.selected_offset_deg:+.0f}deg",
                        *(
                            ("P1/P2 yaw branch preference disagrees",)
                            if yaw_selection.consensus is False
                            else ()
                        ),
                        *(
                            ("baseline yaw branch score margin is low",)
                            if yaw_selection.low_margin
                            else ()
                        ),
                    )
                )
            )
        if (
            measurement_result is not None
            and dict(measurement_result.calibration_quality or {}).get("state")
            == NOMINAL_INTRINSICS_QUALITY
        ):
            warnings = tuple(
                dict.fromkeys(
                    (*warnings, "NOMINAL_UNVERIFIED intrinsics; zero distortion assumed; precision not qualified")
                )
            )
        if triangulation_source == "checkerboard":
            baseline = build_checkerboard_baseline_snapshot(
                intrinsics,
                source=(
                    "checkerboard"
                    if method_id == CHECKERBOARD_STEREO
                    else f"relative-pose:{method_id}"
                ),
            )
            if baseline is None:
                warnings = tuple(dict.fromkeys((*warnings, "selected method R,t unresolved for this pair")))
            elif baseline.warnings:
                warnings = tuple(dict.fromkeys((*warnings, *baseline.warnings)))
        else:
            baseline = None
            if left_gps is not None and right_gps is not None:
                baseline = build_pixhawk_baseline_snapshot(
                    left_gps=left_gps,
                    right_gps=right_gps,
                    selected_left_quaternion_wxyz=(
                        geometry_left_quaternion
                        or (None if left_attitude is None else left_attitude.quaternion_wxyz)
                    ),
                    selected_left_attitude_monotonic_s=(
                        None if left_attitude is None else left_attitude.timestamp_s
                    ),
                    created_monotonic_s=created_monotonic_s,
                    detection_midpoint_monotonic_s=detection_midpoint_s,
                )
                warnings = tuple(dict.fromkeys((*warnings, *baseline.warnings)))
                if baseline.unusable_reason:
                    warnings = tuple(dict.fromkeys((*warnings, baseline.unusable_reason)))
            else:
                warnings = tuple(dict.fromkeys((*warnings, "waiting for live GPS baseline")))

        geometry_sha256 = self._measurement_geometry_hash(
            method_status,
            geometry_kind=geometry_kind,
            intrinsics=intrinsics,
            left_attitude=left_attitude,
            right_attitude=right_attitude,
            geometry_left_quaternion=geometry_left_quaternion,
            geometry_right_quaternion=geometry_right_quaternion,
            yaw_selection=yaw_selection,
            left_gps=left_gps,
            right_gps=right_gps,
        )
        detection_key = detection_pair_key(
            triangulation_source=method_id,
            timestamp_basis=timestamp_basis,
            left=pair.left,
            right=pair.right,
            left_pairing_timestamp_s=left_pairing_timestamp_s,
            right_pairing_timestamp_s=right_pairing_timestamp_s,
        )
        key = (
            *detection_key,
            "geometry_revision_v2",
            method_status.instance_id,
            geometry_sha256,
        )
        cached_attempt = (
            self.panel._last_measurement_attempt
            if use_cache and key == self.panel._last_measurement_attempt_key
            else None
        )
        if cached_attempt is not None and cached_attempt.is_geometry_ready:
            return cached_attempt

        max_time_delta_s = self.panel._yolo_pair_max_delta_s()
        max_vertical_delta_px = self.panel._yolo_vertical_max_delta_px()
        geometry_config = GeometryConfigSnapshot(
            focal_length_px=self.panel.focal_spin.value(),
            max_time_delta_s=max_time_delta_s if math.isfinite(max_time_delta_s) else None,
            max_vertical_delta_px=(
                max_vertical_delta_px if math.isfinite(max_vertical_delta_px) else None
            ),
            require_same_class=True,
            timestamp_basis=timestamp_basis,
            manual_focal_override=self.panel.manual_focal_checkbox.isChecked(),
            require_rtk_fixed_setting=bool(
                self.panel._workstation_setting("require_rtk_fixed", self.panel.settings.require_rtk_fixed)
            ),
            field_quality_gates_advisory=True,
        )
        candidate_attempt_id = snapshot.pair_sequence
        attempt = MeasurementAttempt(
            attempt_id=candidate_attempt_id,
            attempt_session_id=str(
                getattr(self.panel, "_measurement_attempt_generation_id", "")
                or snapshot.measurement_id
            ),
            detection_pair_key=key,
            created_monotonic_s=created_monotonic_s,
            created_wall_s=float(frozen["created_wall_s"]),
            detection_midpoint_monotonic_s=detection_midpoint_s,
            timestamp_basis=timestamp_basis,
            triangulation_source=triangulation_source,
            left_detection=left_detection,
            right_detection=right_detection,
            left_attitude=left_attitude,
            right_attitude=right_attitude,
            left_gps=left_gps,
            right_gps=right_gps,
            intrinsics=intrinsics,
            baseline=baseline,
            geometry_config=geometry_config,
            left_attitude_age_s=left_attitude_age_s,
            right_attitude_age_s=right_attitude_age_s,
            attitude_pair_delta_s=attitude_pair_delta_s,
            relative_roll_deg=None if relative is None else relative.roll_deg,
            relative_pitch_deg=None if relative is None else relative.pitch_deg,
            relative_yaw_deg=None if relative is None else relative.yaw_deg,
            source_quality=(
                "GOOD" if baseline is None or baseline.fix_quality == "NOT_USED" else baseline.fix_quality
            ),
            warnings=warnings,
            measurement_method_id=method_id,
            measurement_method_instance_id=method_status.instance_id,
            measurement_geometry_kind=geometry_kind,
            measurement_geometry_sha256=geometry_sha256,
            measurement_selection_reason=str(
                getattr(self.panel, "_measurement_selection_reason", "selected")
            ),
            measurement_component_sources=method_status.component_sources,
            measurement_started_monotonic_s=measurement_start,
            requested_timestamp_basis=str(
                frozen.get("requested_timestamp_basis") or timestamp_basis
            ),
            timing_fallback_reason=str(
                frozen.get("timing_fallback_reason") or ""
            ),
            geometry_left_quaternion_wxyz=geometry_left_quaternion,
            geometry_right_quaternion_wxyz=geometry_right_quaternion,
            measurement_yaw_source=(
                "gnss_baseline_perpendicular_90deg" if baseline_yaw_method else ""
            ),
            measurement_yaw_selection_mode=(
                "" if yaw_selection is None else yaw_selection.mode
            ),
            measurement_yaw_branch_offset_deg=(
                None if yaw_selection is None else yaw_selection.selected_offset_deg
            ),
            measurement_yaw_baseline_azimuth_deg=(
                None if yaw_selection is None else yaw_selection.baseline_azimuth_deg
            ),
            measurement_yaw_selected_deg=(
                None if yaw_selection is None else yaw_selection.selected_yaw_deg
            ),
            measurement_yaw_p1_reference_deg=(
                None if yaw_selection is None else yaw_selection.p1_reference_yaw_deg
            ),
            measurement_yaw_p2_reference_deg=(
                None if yaw_selection is None else yaw_selection.p2_reference_yaw_deg
            ),
            measurement_yaw_p1_age_s=yaw_p1_age_s,
            measurement_yaw_p2_age_s=yaw_p2_age_s,
            measurement_yaw_score_minus90_deg=(
                None if yaw_selection is None else yaw_selection.score_minus90_deg
            ),
            measurement_yaw_score_plus90_deg=(
                None if yaw_selection is None else yaw_selection.score_plus90_deg
            ),
            measurement_yaw_reverse=(
                False if yaw_selection is None else yaw_selection.reverse
            ),
            measurement_yaw_consensus=(
                None if yaw_selection is None else yaw_selection.consensus
            ),
            measurement_yaw_low_margin=(
                False if yaw_selection is None else yaw_selection.low_margin
            ),
        )
        if cached_attempt is not None:
            previous_missing = set(cached_attempt.missing_geometry_inputs)
            candidate_missing = set(attempt.missing_geometry_inputs)
            if not candidate_missing < previous_missing:
                return cached_attempt
        if use_cache:
            self.panel._last_measurement_attempt_key = key
            self.panel._last_measurement_attempt = attempt
        return attempt

    def _translation_right_in_left_m(
        self, attempt: MeasurementAttempt
    ) -> tuple[float, float, float] | None:
        """Return the immutable attempt translation without consulting latest state."""

        return attempt.translation_right_in_left_m

    def _latest_translation_right_in_left_m_for_diagnostics(self) -> tuple[float, float, float] | None:
        if self.panel._distance_mode() == "checkerboard":
            return self.panel._checkerboard_translation_right_in_left_m()
        left_ext = self.panel.latest_extrinsics.get("left")
        if left_ext is None:
            return None
        baseline = self.panel.latest_rtk_baseline
        if baseline is None:
            return None
        usable, _reason = self.panel._gps_baseline_usable_for_translation(baseline)
        if not usable:
            return None
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        return camera_translation_right_in_left_camera(
            baseline_right_in_left_world_m=world_vector,
            left_body_to_world_q=left_ext.q,
            scale_to_m=None,
        )

    def _alignment_view_camera_translation_right_in_left_m(
        self, baseline: RtkBaselineEstimate
    ) -> tuple[float, float, float] | None:
        left_ext = self.panel.latest_extrinsics.get("left")
        if left_ext is None:
            return None
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        scale_to_m = self.panel.baseline_spin.value() if hasattr(self.panel, "baseline_spin") else None
        return camera_translation_right_in_left_camera(
            baseline_right_in_left_world_m=world_vector,
            left_body_to_world_q=left_ext.q,
            scale_to_m=scale_to_m,
        )

    def _capture_measurement_gnss_contract(
        self, readiness: GnssReadinessSnapshot
    ) -> object | None:
        """Capture the immutable GNSS provenance used by one measurement.

        The baseline hash names the NED vector explicitly: north/east/down is
        the PX4/world position contract, never a camera RDF or stereo-center
        coordinate.  Camera-frame conversion remains in MeasurementAttempt.
        """

        config = self.panel.gnss_operation_manager.config
        if not config.readiness_gate_enabled:
            return None
        base_profile = self.panel.gnss_fixed_base_profile
        base_hash = "" if base_profile is None else str(base_profile.profile_hash)
        vector = self.panel.latest_baseline_vector
        baseline_hash = ""
        if vector is not None:
            baseline_payload = {
                "frame": "NED",
                "axis_order": "north,east,down",
                "north_m": vector.north_m,
                "east_m": vector.east_m,
                "down_m": vector.down_m,
                "timestamp_delta_ms": vector.timestamp_delta_ms,
            }
            baseline_hash = hashlib.sha256(
                json.dumps(baseline_payload, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
        routes = {
            side: f"mavlink:{side}"
            for side in self.panel._gnss_base_rtcm_target_sides()
        }
        selector = getattr(self.panel, "_selected_rover_gps_state", None)
        if callable(selector):
            selected_samples = {
                side: selector(side)
                for side in ("left", "right", "drone")
            }
        else:
            pair_selector = getattr(self.panel, "_selected_baseline_gps_states", None)
            if callable(pair_selector):
                selected_left, selected_right = pair_selector()
            else:
                selected_left = self.panel.latest_gps.get("left")
                selected_right = self.panel.latest_gps.get("right")
            selected_samples = {
                "left": selected_left,
                "right": selected_right,
                "drone": self.panel.latest_gps.get("drone"),
            }
        device_ids = {
            side: str(getattr(sample, "topic", side))
            for side, sample in selected_samples.items()
            if sample is not None
        }
        return self.panel.gnss_operation_manager.capture_measurement_contract(
            readiness,
            base_coordinate_profile_hash=base_hash,
            baseline_profile_hash=baseline_hash,
            device_ids=device_ids,
            route_endpoints=routes,
            datum="WGS84",
        )

    def _measurement_gnss_record_metadata(self) -> dict[str, object]:
        """Return the frozen GNSS contract fields for a recording row."""

        metadata = self.panel.gnss_operation_manager.metadata()
        contract = metadata.get("measurement_contract")
        readiness = metadata.get("readiness")
        if not isinstance(contract, dict):
            return {
                "gnss_contract_id": "",
                "gnss_contract_metadata": "",
                "gnss_readiness_timestamp_s": "",
            }
        return {
            "gnss_contract_id": str(contract.get("gnss_contract_id", "")),
            "gnss_contract_metadata": json.dumps(contract, sort_keys=True, separators=(",", ":")),
            "gnss_readiness_timestamp_s": ""
            if not isinstance(readiness, dict)
            else self.panel._fmt_record_float(readiness.get("timestamp_s"), 6),
        }

    def _geometry_comparison_rows(self, pair) -> tuple[dict[str, object], ...]:
        """Preview F/R/D/Range for every extrinsics source, side by side.

        This never reads ``_distance_mode()`` and never touches
        ``active_intrinsics``/``selected_extrinsics_profile`` — it is a
        read-only comparison for the Measurement tab, independent of which
        source is officially selected for the recorded measurement.
        """

        intrinsics = self.panel.active_intrinsics
        if intrinsics is None:
            return ()
        optical_owner = self.panel._optical_pose_controller()
        try:
            intrinsics_snapshot = optical_owner._intrinsics_snapshot(use_frozen=False)
            active_intrinsics_hash = str(
                intrinsics_snapshot["applicability"].get("profile_sha256") or ""
            )
        except (AttributeError, KeyError, TypeError, ValueError):
            active_intrinsics_hash = ""
        left_pixel = pair.left.center
        right_pixel = pair.right.center
        rows: list[dict[str, object]] = []

        def add_row(
            label: str,
            attempt: object | None,
            *,
            provenance_note: str = "",
            unavailable_status: str = "waiting for inputs",
            metadata: Mapping[str, object] | None = None,
        ) -> None:
            suffix = f" · {provenance_note}" if provenance_note else ""
            base = {
                "schema_version": GEOMETRY_COMPARISON_ANALYSIS_SCHEMA,
                "evidence_namespace": GEOMETRY_COMPARISON_ANALYSIS_NAMESPACE,
                "result_class": "analysis_only",
                "not_measurement_result": True,
                "active_intrinsics_hash": active_intrinsics_hash,
                **dict(metadata or {}),
            }
            if attempt is None:
                rows.append(
                    {
                        **base,
                        "method": label,
                        "forward_m": None,
                        "right_m": None,
                        "down_m": None,
                        "range_m": None,
                        "status": f"{unavailable_status}{suffix}",
                    }
                )
                return
            result = attempt.result
            if result is None:
                rows.append(
                    {
                        **base,
                        "method": label,
                        "forward_m": None,
                        "right_m": None,
                        "down_m": None,
                        "range_m": None,
                        "status": f"{attempt.reason}{suffix}",
                    }
                )
                return
            forward_m, right_m, down_m = result.point_m
            rows.append(
                {
                    **base,
                    "method": label,
                    "forward_m": forward_m,
                    "right_m": right_m,
                    "down_m": down_m,
                    "range_m": result.range_m,
                    "status": f"{attempt.reason}{suffix}",
                }
            )

        def relative_pose_attempt(
            rotation_left_to_right: object, translation_left_to_right_m: object
        ) -> object:
            rotation_right_to_left, translation_right_in_left_m = invert_left_to_right_camera_transform(
                rotation_left_to_right,
                translation_left_to_right_m,
            )
            return triangulate_pixels_with_relative_pose_attempt(
                left_pixel=left_pixel,
                right_pixel=right_pixel,
                left_intrinsics=intrinsics.left,
                right_intrinsics=intrinsics.right,
                rotation_right_to_left=rotation_right_to_left,
                translation_right_in_left_m=translation_right_in_left_m,
            )

        # 1) Pixhawk attitude + live RTK GNSS baseline (always evaluated live,
        # independent of the Distance Mode toggle).
        left_ext = self.panel.latest_extrinsics.get("left")
        right_ext = self.panel.latest_extrinsics.get("right")
        baseline = self.panel.latest_rtk_baseline
        pixhawk_attempt = None
        if left_ext is not None and baseline is not None:
            usable, _reason = self.panel._gps_baseline_usable_for_translation(baseline)
            if usable:
                world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
                translation_right_in_left_m = camera_translation_right_in_left_camera(
                    baseline_right_in_left_world_m=world_vector,
                    left_body_to_world_q=left_ext.q,
                    scale_to_m=None,
                )
                pixhawk_attempt = triangulate_pixels_with_pixhawk_mount_attempt(
                    left_pixel=left_pixel,
                    right_pixel=right_pixel,
                    left_intrinsics=intrinsics.left,
                    right_intrinsics=intrinsics.right,
                    left_body_to_world_q=left_ext.q,
                    right_body_to_world_q=right_ext.q if right_ext is not None else left_ext.q,
                    translation_right_in_left_m=translation_right_in_left_m,
                )
        add_row(
            "Pixhawk + RTK (live)",
            pixhawk_attempt,
            metadata={"source_state": "live"},
        )

        # 2) Checkerboard stereoCalibrate profile, kept in its own slot so a
        # later PnP apply cannot overwrite it here.
        checkerboard_profile = getattr(self.panel, "checkerboard_extrinsics_profile", None)
        checkerboard_attempt = (
            None
            if checkerboard_profile is None
            else relative_pose_attempt(
                checkerboard_profile.extrinsics.rotation_left_to_right,
                checkerboard_profile.extrinsics.translation_left_to_right_m,
            )
        )
        add_row(
            "Checkerboard",
            checkerboard_attempt,
            metadata={
                "source_state": "applied" if checkerboard_profile is not None else "missing",
                "source_profile_id": ""
                if checkerboard_profile is None
                else checkerboard_profile.profile_id,
                "source_profile_hash": ""
                if checkerboard_profile is None
                else checkerboard_profile.content_sha256,
            },
        )

        # 3) The last applied Optical Pose / PnP profile, kept in its own slot.
        applied_profile = getattr(self.panel, "optical_pose_extrinsics_profile", None)
        applied_attempt = (
            None
            if applied_profile is None
            else relative_pose_attempt(
                applied_profile.extrinsics.rotation_left_to_right,
                applied_profile.extrinsics.translation_left_to_right_m,
            )
        )
        add_row(
            "PnP active"
            if applied_profile is None
            else f"PnP active ({applied_profile.method})",
            applied_attempt,
            metadata={
                "source_state": "applied" if applied_profile is not None else "missing",
                "source_profile_id": ""
                if applied_profile is None
                else applied_profile.profile_id,
                "source_profile_hash": ""
                if applied_profile is None
                else applied_profile.content_sha256,
            },
        )

        # 4) Every unapplied PnP candidate (latest solve per method), so an
        # operator can compare Horizon/Mountain/Drone-yaw/Combined solves
        # before deciding whether to apply one.
        candidate_poses = self.panel.workstation_optical_pose_candidate_poses()
        for candidate_row in candidate_poses:
            if bool(candidate_row.get("candidate_applied")):
                continue
            candidate_intrinsics_hash = str(
                candidate_row.get("candidate_intrinsics_hash") or ""
            )
            applicability_reasons: list[str] = []
            if not candidate_intrinsics_hash:
                applicability_reasons.append("candidate K,D identity missing")
            elif not active_intrinsics_hash:
                applicability_reasons.append("active K,D identity missing")
            elif candidate_intrinsics_hash != active_intrinsics_hash:
                applicability_reasons.append("candidate K,D differs from active K,D")
            candidate_applicable = not applicability_reasons
            stereo = stereo_pose_from_absolute(
                candidate_row["left_pose_ned"],
                candidate_row["right_pose_ned"],
            )
            attempt = (
                relative_pose_attempt(
                    [list(row) for row in stereo.rotation_right_from_left_rdf],
                    list(stereo.translation_right_from_left_rdf_m),
                )
                if candidate_applicable
                else None
            )
            # Horizon/Drone-yaw-style solves only optimize a subset of the 6
            # pose DOF; every unselected axis in left/right_pose_ned is a
            # verbatim copy of the prior pose, not something this candidate
            # measured. Label the row so a full-looking F/R/D/Rg is never
            # mistaken for an independent measurement when it is not.
            optimized = tuple(candidate_row.get("optimized_dofs", ()))
            translation_observed = bool(set(optimized) & {"north_m", "east_m", "down_m"})
            observed_text = ",".join(optimized) if optimized else "none"
            if translation_observed:
                note = f"observed={observed_text}"
            else:
                note = (
                    f"observed={observed_text} — Range/F/R/D는 prior 기준선 그대로 상속됨, "
                    "이 방식이 측정한 값 아님"
                )
            applicability = {
                "applicable": candidate_applicable,
                "reasons": applicability_reasons,
            }
            add_row(
                f"PnP candidate: {candidate_row['method']}",
                attempt,
                provenance_note=note,
                unavailable_status=(
                    "analysis blocked: " + "; ".join(applicability_reasons)
                    if applicability_reasons
                    else "waiting for inputs"
                ),
                metadata={
                    "source_state": "candidate_unapplied",
                    "candidate_profile_id": str(candidate_row.get("candidate_id") or ""),
                    "candidate_profile_hash": str(
                        candidate_row.get("candidate_profile_hash") or ""
                    ),
                    "candidate_intrinsics_hash": candidate_intrinsics_hash,
                    "candidate_applied": False,
                    "candidate_applicability": json.dumps(
                        applicability,
                        ensure_ascii=False,
                        allow_nan=False,
                        sort_keys=True,
                        separators=(",", ":"),
                    ),
                    "candidate_quality": json.dumps(
                        dict(candidate_row.get("candidate_quality") or {}),
                        ensure_ascii=False,
                        allow_nan=False,
                        sort_keys=True,
                        separators=(",", ":"),
                    ),
                },
            )

        return tuple(rows)

    def _multi_method_measurement_active(self) -> bool:
        return bool(
            str(getattr(self.panel, "_measurement_id", "") or "")
            and bool(getattr(self.panel, "distance_recording_enabled", False))
            and bool(getattr(self.panel, "_method_evaluation_accepting", True))
        )

    def method_evaluation_pending_count(self) -> int:
        self._poll_method_evaluations()
        coordinator = self._method_evaluation_coordinator
        worker_pairs = 0 if coordinator is None else coordinator.pending_pair_count
        # Primary row admission has independent background ownership and must
        # not hold the sessionless Stop -> next Measurement boundary.
        return int(worker_pairs) + len(self._pending_method_bundle_commits)

    def _method_evaluation_requests_for_pair(
        self,
        pair: object,
    ) -> tuple[MethodEvaluationRequest, ...]:
        frozen = self._freeze_measurement_pair_inputs(pair)
        snapshot = frozen.get("snapshot")
        if not isinstance(snapshot, FrozenMeasurementPairSnapshot):
            raise TypeError("frozen Measurement snapshot unavailable")
        statuses = latest_status_by_method(self._measurement_method_statuses())
        requests: list[MethodEvaluationRequest] = []
        enqueued_ns = time.monotonic_ns()
        for method_id in MEASUREMENT_METHOD_IDS:
            status = statuses.get(method_id)
            attempt_payload: Mapping[str, object] | None = None
            reasons: tuple[str, ...]
            instance_id = f"{method_id}:frozen-input-missing"
            component_sources: tuple[tuple[str, str], ...] = ()
            if status is None:
                reasons = (MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS[method_id],)
            else:
                instance_id = status.instance_id
                component_sources = status.component_sources
                reasons = tuple(status.reasons)
                try:
                    attempt = self._build_measurement_attempt(
                        pair,
                        method_status=status,
                        frozen_inputs=frozen,
                        use_cache=False,
                    )
                    attempt_payload = attempt.canonical_payload()
                except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as exc:
                    reasons = tuple(
                        dict.fromkeys((*reasons, f"frozen method input error: {exc}"))
                    )
            uses_live_gnss = method_id == PIXHAWK_RTK_LIVE or "live_gnss" in dict(
                component_sources
            ).values()
            reused_inputs = snapshot.reused_inputs if uses_live_gnss else ()
            requests.append(
                MethodEvaluationRequest(
                    measurement_id=snapshot.measurement_id,
                    pair_id=snapshot.pair_id,
                    pair_sequence=snapshot.pair_sequence,
                    snapshot_sha256=snapshot.canonical_sha256(),
                    request_generation=snapshot.pair_sequence,
                    method_id=method_id,
                    method_instance_id=instance_id,
                    snapshot_payload=snapshot.canonical_payload(),
                    attempt_payload=attempt_payload,
                    blocked_reasons=reasons,
                    reused_inputs=reused_inputs,
                    enqueued_monotonic_ns=enqueued_ns,
                )
            )
        return tuple(requests)

    def _submit_method_evaluations(self, pair: object) -> None:
        requests = self._method_evaluation_requests_for_pair(pair)
        first = requests[0]
        submitted = getattr(self.panel, "_method_evaluation_submitted", None)
        if not isinstance(submitted, set):
            submitted = set()
            self.panel._method_evaluation_submitted = submitted
        identity = (first.pair_id, first.request_generation)
        if identity in submitted:
            return
        submission = self._ensure_method_evaluation_coordinator().submit(requests)
        submitted.add(identity)
        if submission.backpressure_bundle is not None:
            self._commit_method_evaluation_bundle(submission.backpressure_bundle)
            self.panel._set_distance_text(
                "거리: method worker queue 포화 (20 pairs)",
                quality="WARNING",
                detail="원본 pair와 BACKPRESSURE_BLOCKED 결과는 저장했습니다.",
            )

    @staticmethod
    def _primary_outcome_key(
        outcome: MethodEvaluationOutcome,
    ) -> tuple[str, str, int]:
        return (
            str(outcome.measurement_id),
            str(outcome.pair_id),
            int(outcome.request_generation),
        )

    @staticmethod
    def _failed_primary_outcome(
        outcome: MethodEvaluationOutcome,
        reason: str,
    ) -> MethodEvaluationOutcome:
        message = str(reason)
        return replace(
            outcome,
            status="FAILED",
            result={},
            reasons=(message,),
            warnings=tuple(dict.fromkeys((*outcome.warnings, message))),
        )

    def _primary_outcome_is_current(
        self,
        outcome: MethodEvaluationOutcome,
    ) -> bool:
        if not bool(getattr(self.panel, "distance_recording_enabled", False)):
            return False
        sessionless = not bool(
            getattr(self.panel, "_measurement_session_hierarchy_enabled", True)
        )
        if not sessionless:
            raw_measurement_id = getattr(self.panel, "_measurement_id", None)
            if raw_measurement_id is None:
                # Lightweight legacy embedders predate generation ownership.
                # The production panel always exposes _measurement_id.
                return True
            current_measurement_id = str(
                raw_measurement_id or ""
            )
            return bool(
                current_measurement_id
                and outcome.measurement_id == current_measurement_id
            )
        current_generation = str(
            getattr(self.panel, "_measurement_run_generation", "")
            or getattr(self.panel, "_measurement_id", "")
            or ""
        )
        run_state = str(
            getattr(self.panel, "_measurement_run_state", "IDLE") or "IDLE"
        ).upper()
        return bool(
            run_state == "RUNNING"
            and current_generation
            and outcome.measurement_id == current_generation
        )

    def _normalize_primary_outcome(
        self,
        outcome: MethodEvaluationOutcome,
        *,
        expected_method_id: str = PIXHAWK_RTK_LIVE,
    ) -> MethodEvaluationOutcome:
        key = self._primary_outcome_key(outcome)
        outcome_sha256 = outcome.canonical_sha256()
        previous_sha256 = self._primary_outcome_hash_by_key.get(key)
        if previous_sha256 is not None and previous_sha256 != outcome_sha256:
            logger = getattr(self.panel, "_log", None)
            if callable(logger):
                logger(
                    "[측정] conflicting primary outcome ignored after first "
                    f"accepted identity: pair={outcome.pair_id}"
                )
            cached = self._primary_normalized_outcome_by_key.get(key)
            return cached if cached is not None else self._failed_primary_outcome(
                outcome,
                "conflicting primary method outcome for one request identity",
            )
        self._primary_outcome_hash_by_key.setdefault(key, outcome_sha256)
        cached = self._primary_normalized_outcome_by_key.get(key)
        if cached is not None:
            return cached
        normalized = outcome
        try:
            if outcome.method_id != expected_method_id:
                raise ValueError(
                    f"primary outcome method is not {expected_method_id}"
                )
            if outcome.status in {"VALID", "ESTIMATED"}:
                if not isinstance(outcome.attempt_payload, Mapping):
                    raise ValueError("successful primary outcome has no attempt payload")
                attempt = MeasurementAttempt.from_canonical_payload(
                    outcome.attempt_payload
                )
                if attempt.canonical_sha256() != outcome.attempt_sha256:
                    raise ValueError("primary outcome attempt hash mismatch")
                if attempt.measurement_method_id != expected_method_id:
                    raise ValueError(
                        f"primary attempt method is not {expected_method_id}"
                    )
                if attempt.measurement_geometry_sha256 != outcome.geometry_sha256:
                    raise ValueError("primary outcome geometry hash mismatch")
                self._method_history_payload(outcome, attempt)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            normalized = self._failed_primary_outcome(
                outcome,
                f"primary outcome validation failed: {exc}",
            )
        self._primary_normalized_outcome_by_key[key] = normalized
        return normalized

    def _commit_primary_method_outcome(
        self,
        outcome: MethodEvaluationOutcome,
        *,
        commit_source: str,
        expected_method_id: str = PIXHAWK_RTK_LIVE,
    ) -> tuple[MethodEvaluationOutcome, bool, str]:
        normalized = self._normalize_primary_outcome(
            outcome,
            expected_method_id=expected_method_id,
        )
        key = self._primary_outcome_key(normalized)
        current = self._primary_outcome_is_current(normalized)
        if not current:
            return normalized, False, "NOT_REQUESTED"
        if normalized.status not in {"VALID", "ESTIMATED"}:
            return normalized, True, "NOT_REQUESTED"
        if key in self._primary_measurement_persisted:
            self._remove_pending_primary_measurement(key)
            return normalized, True, "QUEUED"

        try:
            attempt = MeasurementAttempt.from_canonical_payload(
                normalized.attempt_payload
            )
            history_payload = self._method_history_payload(normalized, attempt)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            failed = self._failed_primary_outcome(
                normalized,
                f"primary persistence payload invalid: {exc}",
            )
            self._primary_normalized_outcome_by_key[key] = failed
            return failed, True, "NOT_REQUESTED"

        publish_history = key not in self._primary_history_projected
        outcome_sha256 = normalized.canonical_sha256()
        metadata = {
            "runtime_generation": normalized.measurement_id,
            "method_evaluation_pair_id": normalized.pair_id,
            "method_evaluation_pair_sequence": normalized.pair_sequence,
            "method_evaluation_request_generation": normalized.request_generation,
            "method_evaluation_snapshot_sha256": normalized.snapshot_sha256,
            "primary_outcome_status": normalized.status,
            "primary_outcome_sha256": outcome_sha256,
            "primary_committed_before_bundle": commit_source == "early_outcome",
            "primary_commit_source": str(commit_source),
        }
        method = f"{history_payload['method']}-{expected_method_id}"
        committer = getattr(
            self.panel,
            "_commit_primary_distance_outcome",
            None,
        )
        if callable(committer):
            commit_result = committer(
                estimate=history_payload["estimate"],
                distance_m=float(history_payload["distance_m"]),
                method=method,
                baseline_source=attempt.baseline_source,
                baseline_m=attempt.baseline_m,
                point_m=history_payload["point_m"],
                raw_point_m=history_payload["point_m"],
                raw_point_left_m=history_payload["source_point_left_rdf_m"],
                measurement_attempt=attempt,
                outcome_metadata=metadata,
                publish_history=publish_history,
            )
            history_added = bool(getattr(commit_result, "history_added", False))
            row_commit = getattr(commit_result, "row_commit", None)
            enqueue_admitted = bool(
                getattr(row_commit, "enqueue_admitted", False)
            )
        else:
            legacy_append = getattr(self.panel, "_append_distance_history", None)
            enqueue_admitted = bool(
                callable(legacy_append)
                and legacy_append(
                    estimate=history_payload["estimate"],
                    distance_m=float(history_payload["distance_m"]),
                    method=method,
                    baseline_source=attempt.baseline_source,
                    baseline_m=attempt.baseline_m,
                    point_m=history_payload["point_m"],
                    raw_point_m=history_payload["point_m"],
                    raw_point_left_m=history_payload["source_point_left_rdf_m"],
                    measurement_attempt=attempt,
                )
            )
            history_added = enqueue_admitted
        if history_added:
            self._primary_history_projected.add(key)
        if enqueue_admitted:
            self._primary_measurement_persisted.add(key)
            self._remove_pending_primary_measurement(key)
            return normalized, True, "QUEUED"

        first_rejection = key not in self._pending_primary_measurement_commits
        retained = self._retain_pending_primary_measurement(key, normalized)
        if retained:
            self._ensure_method_evaluation_timer()
        logger = getattr(self.panel, "_log", None)
        if first_rejection and callable(logger):
            logger(
                "[측정] PX4+RTK 대표 결과 저장 enqueue 실패; "
                f"background에서 재시도합니다. pair={normalized.pair_id}"
            )
        return normalized, True, "REJECTED"

    def _persist_late_primary_measurement(
        self,
        outcome: MethodEvaluationOutcome,
    ) -> bool:
        """Persist a retired primary result without touching the next run's rows/UI."""

        writer = getattr(self.panel, "append_workstation_data", None)
        if not callable(writer):
            return False
        current_generation = str(
            getattr(self.panel, "_measurement_run_generation", "")
            or getattr(self.panel, "_measurement_id", "")
            or ""
        )
        primary_record = outcome.to_record()
        primary_canonical_json = json.dumps(
            primary_record,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        record = {
            "event_type": "late_primary_measurement_outcome",
            "record_kind": "primary_measurement_evidence",
            "runtime_generation": str(outcome.measurement_id),
            "measurement_id": str(outcome.measurement_id),
            "method_evaluation_pair_id": str(outcome.pair_id),
            "method_evaluation_pair_sequence": int(outcome.pair_sequence),
            "method_evaluation_request_generation": int(
                outcome.request_generation
            ),
            "primary_outcome_status": str(outcome.status),
            "primary_outcome_sha256": outcome.canonical_sha256(),
            "primary_outcome": primary_record,
            "primary_outcome_canonical_json": primary_canonical_json,
            "primary_persistence_mode": "late_direct_retry",
            "late": True,
            "current_runtime_generation": current_generation,
        }
        recording_owner = getattr(
            self.panel,
            "_recording_controller_owner",
            None,
        )
        compact_persister = getattr(
            recording_owner,
            "_persist_compact_primary_measurement",
            None,
        )
        if callable(compact_persister):
            try:
                attempt = MeasurementAttempt.from_canonical_payload(
                    outcome.attempt_payload
                )
                return bool(
                    compact_persister(
                        tuple(attempt.detection_pair_key),
                        record,
                    )
                )
            except (KeyError, OSError, RuntimeError, TypeError, ValueError):
                return False
        try:
            return bool(writer("measurements", record, priority=True))
        except (OSError, RuntimeError, TypeError, ValueError):
            return False

    def _retry_pending_primary_measurements(self) -> None:
        """Retry primary evidence independently from the already-saved bundle."""

        if not self._pending_primary_measurement_commits:
            return
        logger = getattr(self.panel, "_log", None)
        retry_batch = tuple(
            self._pending_primary_measurement_commits.items()
        )[: max(1, int(PRIMARY_MEASUREMENT_RETRY_BUDGET))]
        for key, outcome in retry_batch:
            self._pending_primary_measurement_commits.pop(key, None)
            if key in self._primary_measurement_persisted:
                self._remove_pending_primary_measurement(key)
                continue
            # Reinsert before attempting so a rejection retains ownership while
            # moving this key to the tail for round-robin fairness.
            self._pending_primary_measurement_commits[key] = outcome
            try:
                if self._primary_outcome_is_current(outcome):
                    self._commit_primary_method_outcome(
                        outcome,
                        commit_source="background_retry",
                    )
                    continue
                if self._persist_late_primary_measurement(outcome):
                    self._primary_measurement_persisted.add(key)
                    self._remove_pending_primary_measurement(key)
            except Exception as exc:  # noqa: BLE001 - keep the retry timer alive.
                if callable(logger):
                    logger(f"[측정] primary 저장 재시도 실패: {exc}")
        if (
            not self._pending_primary_measurement_commits
            and not self._pending_method_bundle_commits
            and self._method_evaluation_coordinator is None
        ):
            self._stop_method_evaluation_timer()

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
        if outcome.status in {"VALID", "ESTIMATED"} and isinstance(triangulated, Mapping):
            distance_m = triangulated.get("range_m")
            try:
                distance_text = f"{float(distance_m):.2f} m"
            except (TypeError, ValueError):
                distance_text = "--"
            reused = " · REUSED" if outcome.reused_inputs else ""
            method_label = MEASUREMENT_METHOD_LABELS.get(
                outcome.method_id,
                outcome.method_id,
            )
            self.panel._set_distance_text(
                f"거리: {distance_text} ({method_label} worker){reused}{suffix}",
                quality=(
                    "WARNING"
                    if persistence_rejected
                    else "ESTIMATED"
                    if outcome.status == "ESTIMATED"
                    else "GOOD"
                ),
            )
            return
        reason = outcome.reasons[0] if outcome.reasons else outcome.status
        self.panel._set_distance_text(
            f"거리: Pixhawk + GNSS/RTK {outcome.status} · {reason}{suffix}",
            quality="ERROR" if outcome.status == "FAILED" else "WARNING",
        )

    def _update_method_evaluation_counters(
        self,
        bundle: MethodEvaluationBundle,
    ) -> None:
        counters = getattr(self.panel, "_measurement_method_result_counters", None)
        if not isinstance(counters, dict):
            counters = {
                method_id: {"VALID": 0, "BLOCKED": 0, "FAILED": 0}
                for method_id in MEASUREMENT_METHOD_IDS
            }
            self.panel._measurement_method_result_counters = counters
        for outcome in bundle.outcomes:
            bucket = (
                "VALID"
                if outcome.status in {"VALID", "ESTIMATED"}
                else "BLOCKED"
                if outcome.status in {"BLOCKED", "BACKPRESSURE_BLOCKED"}
                else "FAILED"
            )
            counters.setdefault(
                outcome.method_id,
                {"VALID": 0, "BLOCKED": 0, "FAILED": 0},
            )[bucket] += 1

    @staticmethod
    def _method_bundle_key(
        bundle: MethodEvaluationBundle,
    ) -> tuple[str, str, int]:
        return (
            str(bundle.measurement_id),
            str(bundle.pair_id),
            int(bundle.request_generation),
        )

    def _method_bundle_is_current(
        self,
        bundle: MethodEvaluationBundle,
    ) -> tuple[bool, str]:
        sessionless = not bool(
            getattr(self.panel, "_measurement_session_hierarchy_enabled", True)
        )
        current_generation = str(
            getattr(self.panel, "_measurement_run_generation", "") or ""
        )
        run_state = str(
            getattr(self.panel, "_measurement_run_state", "IDLE") or "IDLE"
        ).upper()
        if not sessionless:
            raw_measurement_id = getattr(self.panel, "_measurement_id", None)
            if raw_measurement_id is None:
                return bool(
                    getattr(self.panel, "distance_recording_enabled", False)
                ), str(bundle.measurement_id)
            hierarchy_generation = str(
                raw_measurement_id
                or current_generation
                or ""
            )
            return bool(
                getattr(self.panel, "distance_recording_enabled", False)
                and hierarchy_generation
                and bundle.measurement_id == hierarchy_generation
            ), hierarchy_generation
        return bool(
            bundle.measurement_id == current_generation and run_state == "RUNNING"
        ), current_generation

    def _method_bundle_record(
        self,
        bundle: MethodEvaluationBundle,
        *,
        current: bool,
        current_generation: str,
    ) -> dict[str, object]:
        record = dict(bundle.to_record())
        if not current:
            record.update(
                {
                    "late": True,
                    "current_runtime_generation": str(current_generation),
                }
            )
        return record

    @staticmethod
    def _write_admitted(result: object) -> bool:
        return True if result is None else bool(result)

    @staticmethod
    def _method_bundle_row_owner_usable(
        owner: object,
        index: int | None,
    ) -> bool:
        if index is None:
            return False
        try:
            if index < 0 or index >= len(owner):  # type: ignore[arg-type]
                return False
        except (AttributeError, RuntimeError, TypeError, ValueError):
            return False

        store = getattr(owner, "store", None)
        snapshot_reader = getattr(store, "snapshot", None)
        if callable(snapshot_reader):
            try:
                snapshot = snapshot_reader()
            except (KeyError, OSError, RuntimeError, TypeError, ValueError):
                return False
            if isinstance(snapshot, Mapping):
                lifecycle = str(snapshot.get("lifecycle_state") or "").upper()
                if lifecycle and lifecycle != "OPEN":
                    return False
                if snapshot.get("writer_alive") is False:
                    return False

        try:
            write_error = getattr(owner, "write_error", None)
        except (KeyError, OSError, RuntimeError, TypeError, ValueError):
            return False
        if write_error:
            return False
        return True

    def _persist_method_evaluation_bundle(
        self,
        bundle: MethodEvaluationBundle,
    ) -> bool:
        key = self._method_bundle_key(bundle)
        current, current_generation = self._method_bundle_is_current(bundle)
        record = self._method_bundle_record(
            bundle,
            current=current,
            current_generation=current_generation,
        )

        def append_direct() -> bool:
            writer = getattr(self.panel, "append_workstation_data", None)
            try:
                return bool(
                    callable(writer)
                    and writer(
                        "method_evaluations",
                        record,
                        priority=True,
                    )
                )
            except (OSError, RuntimeError, TypeError, ValueError):
                return False

        pending = self._pending_method_bundle_commits.get(key)
        pending_candidate: _PendingMethodBundleCommit | None = None
        if pending is not None:
            admitted = False
            if pending.row_owner is None:
                # Once an owner is retired, retries stay on the direct writer.
                # Re-appending here would create one in-memory row per timer tick.
                admitted = append_direct()
            else:
                owner = pending.row_owner
                index = pending.row_index
                owner_usable = self._method_bundle_row_owner_usable(owner, index)
                owner_unavailable = not owner_usable
                if owner_usable:
                    atomic_replacer = getattr(owner, "replace_with_admission", None)
                    replacer = getattr(owner, "replace", None)
                    try:
                        if callable(atomic_replacer):
                            replacement = atomic_replacer(
                                index,
                                record,
                                lease=pending.row_lease,
                            )
                            replacement_state = str(
                                getattr(replacement, "state", "") or ""
                            ).upper()
                            admitted = replacement_state == ROW_WRITE_ADMITTED
                            owner_unavailable = (
                                replacement_state == ROW_WRITE_UNAVAILABLE
                            )
                        elif callable(replacer):
                            admitted = self._write_admitted(replacer(index, record))
                        else:
                            owner[index] = dict(record)  # type: ignore[index]
                            admitted = True
                    except (IndexError, OSError, RuntimeError, TypeError, ValueError):
                        admitted = False
                        owner_unavailable = True
                if not admitted and owner_unavailable:
                    pending.row_owner = None
                    pending.row_index = None
                    pending.row_lease = None
                    admitted = append_direct()
        elif current:
            owner = getattr(self.panel, "method_evaluation_bundle_rows", None)
            atomic_appender = getattr(owner, "append_with_admission", None)
            appender = getattr(owner, "append", None)
            if callable(atomic_appender):
                index = len(owner)
                appended = atomic_appender(record)
                append_state = str(getattr(appended, "state", "") or "").upper()
                admitted = append_state == ROW_WRITE_ADMITTED
                owner_unavailable = append_state == ROW_WRITE_UNAVAILABLE
                row_index = getattr(appended, "row_index", index)
                row_lease = getattr(appended, "lease", None)
                if not admitted:
                    if owner_unavailable:
                        admitted = append_direct()
                    else:
                        pending_candidate = _PendingMethodBundleCommit(
                            bundle=bundle,
                            row_owner=owner,
                            row_index=row_index,
                            row_lease=row_lease,
                        )
            elif callable(appender):
                index = len(owner)
                admitted = self._write_admitted(appender(record))
                if not admitted:
                    pending_candidate = _PendingMethodBundleCommit(
                        bundle=bundle,
                        row_owner=owner,
                        row_index=index,
                    )
            else:
                admitted = False
        else:
            admitted = append_direct()

        if admitted:
            self._remove_pending_method_bundle(key)
            self._persisted_method_bundle_keys.add(key)
            committed = getattr(self.panel, "_method_evaluation_committed", None)
            current_after_write, _generation_after_write = (
                self._method_bundle_is_current(bundle)
            )
            if current_after_write and isinstance(committed, set):
                committed.add((bundle.pair_id, bundle.request_generation))
            return True

        existing = pending_candidate or self._pending_method_bundle_commits.get(key)
        if existing is None:
            existing = _PendingMethodBundleCommit(bundle=bundle)
        else:
            existing.bundle = bundle
        self._retain_pending_method_bundle(
            key,
            existing,
        )
        return False

    def _retry_pending_method_bundles(self) -> None:
        if not self._pending_method_bundle_commits:
            return
        logger = getattr(self.panel, "_log", None)
        retry_batch = tuple(self._pending_method_bundle_commits.items())[
            : max(1, int(METHOD_BUNDLE_RETRY_BUDGET))
        ]
        for key, pending in retry_batch:
            self._pending_method_bundle_commits.pop(key, None)
            self._pending_method_bundle_commits[key] = pending
            try:
                self._commit_method_evaluation_bundle(pending.bundle)
            except Exception as exc:  # noqa: BLE001 - keep the timer alive.
                if callable(logger):
                    logger(f"[측정] method bundle 저장 재시도 실패: {exc}")
        if (
            not self._pending_method_bundle_commits
            and not self._pending_primary_measurement_commits
            and self._method_evaluation_coordinator is None
        ):
            self._stop_method_evaluation_timer()

    def _commit_method_evaluation_bundle(
        self,
        bundle: MethodEvaluationBundle,
    ) -> None:
        bundle_key = self._method_bundle_key(bundle)
        if bundle_key in self._persisted_method_bundle_keys:
            return
        committed = getattr(self.panel, "_method_evaluation_committed", None)
        if not isinstance(committed, set):
            committed = set()
            self.panel._method_evaluation_committed = committed
        identity = (bundle.pair_id, bundle.request_generation)
        if identity in committed:
            return
        selected_method_id = str(
            getattr(self.panel, "_measurement_recording_method_id", "")
            or PIXHAWK_RTK_LIVE
        )
        if selected_method_id not in MEASUREMENT_METHOD_IDS:
            selected_method_id = PIXHAWK_RTK_LIVE
        selected_index = MEASUREMENT_METHOD_IDS.index(selected_method_id)
        selected_outcome = bundle.outcomes[selected_index]
        primary, current, persistence_state = self._commit_primary_method_outcome(
            selected_outcome,
            commit_source="bundle_fallback",
            expected_method_id=selected_method_id,
        )
        if primary != selected_outcome:
            outcomes = list(bundle.outcomes)
            outcomes[selected_index] = primary
            bundle = replace(bundle, outcomes=tuple(outcomes))

        if not self._persist_method_evaluation_bundle(bundle):
            return
        current_after_write, _generation_after_write = (
            self._method_bundle_is_current(bundle)
        )
        if not current_after_write:
            return
        self.panel._last_method_evaluation_bundle = bundle
        self._update_method_evaluation_counters(bundle)
        primary_key = self._primary_outcome_key(primary)
        if current and (
            self._primary_displayed_key == primary_key
            or primary.pair_sequence >= self._primary_display_high_water
        ):
            self._primary_display_high_water = max(
                self._primary_display_high_water,
                primary.pair_sequence,
            )
            self._primary_displayed_key = primary_key
            self._display_primary_method_outcome(
                primary,
                waiting_for_barrier=False,
                persistence_state=persistence_state,
            )

    @staticmethod
    def _method_bundle_with_primary_failure(
        bundle: MethodEvaluationBundle,
        reason: str,
        *,
        primary_method_id: str = PIXHAWK_RTK_LIVE,
    ) -> MethodEvaluationBundle:
        outcomes = list(bundle.outcomes)
        primary_index = MEASUREMENT_METHOD_IDS.index(primary_method_id)
        primary = outcomes[primary_index]
        outcomes[primary_index] = replace(
            primary,
            status="FAILED",
            result={},
            reasons=(str(reason),),
            warnings=tuple(dict.fromkeys((*primary.warnings, str(reason)))),
        )
        return replace(bundle, outcomes=tuple(outcomes))

    @staticmethod
    def _method_history_payload(
        outcome: MethodEvaluationOutcome,
        attempt: MeasurementAttempt,
    ) -> dict[str, object]:
        result = outcome.result
        match = result.get("match")
        triangulated = result.get("triangulated")
        estimate_payload = result.get("estimate")
        if not all(
            isinstance(value, Mapping)
            for value in (match, triangulated, estimate_payload)
        ):
            raise ValueError("worker result is missing match/triangulated/estimate")

        def finite_number(mapping: Mapping[str, object], name: str) -> float:
            value = float(mapping[name])
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            return value

        def vector3(mapping: Mapping[str, object], name: str) -> tuple[float, float, float]:
            raw = mapping[name]
            if not isinstance(raw, (tuple, list)) or len(raw) != 3:
                raise ValueError(f"{name} must contain three values")
            result_vector = tuple(float(value) for value in raw)
            if not all(math.isfinite(value) for value in result_vector):
                raise ValueError(f"{name} must be finite")
            return result_vector  # type: ignore[return-value]

        point_m = vector3(triangulated, "point_m")
        source_raw = triangulated.get("source_point_left_rdf_m")
        source_point = (
            None
            if source_raw is None
            else vector3(
                {"source_point_left_rdf_m": source_raw},
                "source_point_left_rdf_m",
            )
        )
        distance_m = finite_number(triangulated, "range_m")
        estimate = StereoEstimate(
            distance_m=finite_number(estimate_payload, "distance_m"),
            disparity_px=finite_number(match, "disparity_px"),
            time_delta_s=finite_number(match, "time_delta_s"),
            vertical_delta_px=finite_number(match, "vertical_delta_px"),
            left=attempt.left_detection,  # type: ignore[arg-type]
            right=attempt.right_detection,  # type: ignore[arg-type]
            left_extrinsic=attempt.left_attitude,
            right_extrinsic=attempt.right_attitude,
            extrinsic_time_delta_s=(
                None
                if attempt.left_attitude is None or attempt.right_attitude is None
                else abs(
                    attempt.left_attitude.timestamp_s
                    - attempt.right_attitude.timestamp_s
                )
            ),
            left_attitude_age_s=attempt.left_attitude_age_s,
            right_attitude_age_s=attempt.right_attitude_age_s,
            attitude_pair_delta_s=attempt.attitude_pair_delta_s,
            relative_roll_deg=attempt.relative_roll_deg,
            relative_pitch_deg=attempt.relative_pitch_deg,
            relative_yaw_deg=attempt.relative_yaw_deg,
            sync_quality=str(estimate_payload.get("sync_quality") or match.get("sync_quality") or "unknown"),
            warnings=tuple(str(value) for value in estimate_payload.get("warnings", ()) or ()),
        )
        return {
            "estimate": estimate,
            "distance_m": distance_m,
            "method": str(triangulated.get("method") or "unknown"),
            "point_m": point_m,
            "source_point_left_rdf_m": source_point,
        }

    def _poll_method_evaluations(self) -> None:
        self._retry_pending_primary_measurements()
        self._retry_pending_method_bundles()
        coordinator = self._method_evaluation_coordinator
        if coordinator is None:
            return
        try:
            polled = coordinator.poll()
        except Exception as exc:  # noqa: BLE001 - this method is a 20 ms Qt timer slot.
            logger = getattr(self.panel, "_log", None)
            if callable(logger):
                logger(f"[측정] method worker poll 실패(타이머 유지): {exc}")
            return
        for primary in polled.primary_outcomes:
            selected_method_id = str(
                getattr(self.panel, "_measurement_recording_method_id", "")
                or PIXHAWK_RTK_LIVE
            )
            if selected_method_id != PIXHAWK_RTK_LIVE:
                # The fixed Pixhawk worker remains the early admission path;
                # a non-Pixhawk selected method is committed only from the
                # completed bundle after its own comparison worker finishes.
                continue
            try:
                primary, current, persistence_state = (
                    self._commit_primary_method_outcome(
                        primary,
                        commit_source="early_outcome",
                    )
                )
                if current and primary.pair_sequence >= self._primary_display_high_water:
                    self._primary_display_high_water = primary.pair_sequence
                    self._primary_displayed_key = self._primary_outcome_key(primary)
                    self._display_primary_method_outcome(
                        primary,
                        waiting_for_barrier=True,
                        persistence_state=persistence_state,
                    )
            except Exception as exc:  # noqa: BLE001 - isolate one malformed outcome.
                logger = getattr(self.panel, "_log", None)
                if callable(logger):
                    logger(f"[측정] primary method 저장/표시 실패: {exc}")
        for bundle in polled.completed_bundles:
            try:
                self._commit_method_evaluation_bundle(bundle)
            except Exception as exc:  # noqa: BLE001 - never escape the QTimer callback.
                failed_bundle = self._method_bundle_with_primary_failure(
                    bundle,
                    f"GUI method bundle commit failure: {type(exc).__name__}: {exc}",
                    primary_method_id=str(
                        getattr(self.panel, "_measurement_recording_method_id", "")
                        or PIXHAWK_RTK_LIVE
                    ),
                )
                logger = getattr(self.panel, "_log", None)
                if callable(logger):
                    logger(f"[측정] method bundle FAILED 처리: {exc}")
                try:
                    self._commit_method_evaluation_bundle(failed_bundle)
                except Exception as nested_exc:  # noqa: BLE001 - last-resort isolation.
                    if callable(logger):
                        logger(f"[측정] FAILED bundle 기록도 실패: {nested_exc}")

    def _refresh_distance(self) -> None:
        self.panel.geometry_comparison_rows = ()
        if self.panel._rtsp_urls_are_same_stream():
            self.panel._clear_measurement_pair_display()
            self.panel._clear_triangulation_quality("좌/우 RTSP URL이 동일해 3D ray 품질을 표시할 수 없습니다.")
            self.panel._set_distance_text(
                "거리: 좌/우 RTSP URL이 동일해 측정 차단",
                quality="ERROR",
                detail="그래프 미표시: 좌/우가 같은 스트림이라 유효한 매칭 쌍이 없습니다.",
            )
            return
        pair = self.panel._matched_detection_pair()
        if pair is None:
            self.panel._clear_measurement_pair_display()
            wait_status = self.panel._detection_wait_status()
            self.panel._clear_triangulation_quality(wait_status.removeprefix("거리: "))
            self.panel._set_distance_text(
                wait_status,
                quality="WARNING",
                detail=self.panel._detection_wait_detail(),
            )
            return
        try:
            self.panel.geometry_comparison_rows = self.panel._geometry_comparison_rows(pair)
        except (TypeError, ValueError):
            # A comparison-only preview must never block the official
            # measurement pipeline below it.
            self.panel.geometry_comparison_rows = ()
        if self._multi_method_measurement_active():
            selected_method_status = self._select_measurement_method_for_attempt()
            if selected_method_status is not None and not str(
                getattr(self.panel, "_measurement_recording_method_id", "") or ""
            ):
                self.panel._measurement_recording_method_id = selected_method_status.method_id
            if self.panel._measurement_display_mode() == "detection":
                self.panel._show_measurement_pair(pair.left, pair.right)
            else:
                self.panel._clear_measurement_pair_display()
            try:
                self._submit_method_evaluations(pair)
            except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as exc:
                self.panel._clear_triangulation_quality("method snapshot 생성 실패")
                self.panel._set_distance_text(
                    f"거리: method snapshot 생성 실패 | {exc}",
                    quality="ERROR",
                )
            return
        selected_method_status = self._select_measurement_method_for_attempt()
        if selected_method_status is None:
            record_detail = "해당 matched pair를 geometry 미준비 상태로 기록했습니다."
            try:
                self._record_no_ready_measurement_pair(pair)
            except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as exc:
                # Recording the blocked-pair evidence entry must never stop the
                # refresh loop itself; a missing timing decision here just means
                # this one pair goes unlogged instead of crashing the loop.
                record_detail = f"blocked-pair 기록 실패(계속 진행): {exc}"
            self.panel._clear_triangulation_quality("준비된 측정 방식 없음")
            self.panel._set_distance_text(
                "거리: 준비된 측정 방식 없음",
                quality="WARNING",
                detail=record_detail,
            )
            return
        gnss_measurement_advisories: tuple[str, ...] = ()
        if (
            selected_method_status is not None
            and selected_method_status.requires_live_gnss
            and self.panel.gnss_operation_gate_active
        ):
            readiness = self.panel.gnss_operation_manager.readiness()
            config = self.panel.gnss_operation_manager.config
            if config.readiness_gate_enabled and (readiness is None or not readiness.ready):
                reasons = ("GNSS readiness snapshot 대기",) if readiness is None else readiness.reasons
                reason_text = "; ".join(reasons[:3]) or self.panel.gnss_operation_manager.state.value
                gnss_measurement_advisories = tuple(reasons) or (reason_text,)
            if readiness is not None:
                try:
                    contract = self.panel._capture_measurement_gnss_contract(readiness)
                    if self.panel.gnss_operation_manager.state == GnssOperationState.READY:
                        self.panel.gnss_operation_manager.begin_measurement(contract, readiness)
                    elif self.panel.gnss_operation_manager.state == GnssOperationState.MEASURING:
                        if not self.panel.gnss_operation_manager.validate_measurement_contract(contract, readiness):
                            raise RuntimeError("GNSS measurement contract changed")
                except (RuntimeError, ValueError) as exc:
                    gnss_measurement_advisories = tuple(
                        dict.fromkeys((*gnss_measurement_advisories, str(exc)))
                    )
        if self.panel._measurement_display_mode() == "detection":
            self.panel._show_measurement_pair(pair.left, pair.right)
        else:
            self.panel._clear_measurement_pair_display()

        try:
            attempt = self.panel._build_measurement_attempt(pair)
        except (TypeError, ValueError) as exc:
            self.panel._clear_triangulation_quality("측정 입력 snapshot 생성 실패")
            self.panel._set_distance_text(
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

        preview_result = self._preview_measurement_result(attempt, pair)
        if preview_result is None:
            self.panel._clear_triangulation_quality("measurement worker 계산 중")
            self.panel._set_distance_text(
                "거리: measurement worker 계산 중",
                quality="WARNING",
                detail="GUI는 계산하지 않고 latest worker 결과를 기다립니다.",
            )
            return
        if preview_result.error or preview_result.geometry_result is None:
            self.panel._clear_triangulation_quality("measurement worker 실패")
            self.panel._set_distance_text(
                f"거리: measurement worker 실패 | {preview_result.error}",
                quality="ERROR",
            )
            return
        attempt = preview_result.attempt or attempt
        self.panel._latest_preview_measurement_result = preview_result
        self.panel._latest_preview_pair_identity = (
            str(pair.left.timestamp_metadata.get("frame_key") or pair.left.timestamp_s),
            str(pair.right.timestamp_metadata.get("frame_key") or pair.right.timestamp_s),
        )
        baseline_m = attempt.baseline_m
        baseline_source = attempt.baseline_source
        geometry_result = preview_result.geometry_result
        match = geometry_result.match
        triangulated = geometry_result.triangulated
        estimate = geometry_result.estimate
        combined_warnings = geometry_result.warnings
        if match is None:
            self.panel._record_unresolved_pair(
                attempt,
                status="matched_stereo_geometry_invalid",
                warnings=combined_warnings,
            )
            self.panel._clear_triangulation_quality("유효한 스테레오 기하 대기 중")
            self.panel._set_distance_text(
                "거리: 유효한 스테레오 기하 대기 중",
                quality="WARNING",
                detail=self.panel._stereo_geometry_wait_detail(pair),
            )
            return
        if triangulated is None:
            self.panel._record_unresolved_pair(
                attempt,
                status="matched_triangulation_pending",
                warnings=combined_warnings,
            )
            warning_text = f" | {'; '.join(combined_warnings)}" if combined_warnings else ""
            self.panel._clear_triangulation_quality("3D ray 기하 대기 중")
            self.panel._set_distance_text(
                f"거리: 3D ray 기하 대기 중 ({self.panel._triangulation_extrinsics_source_label()}){warning_text}",
                quality="WARNING",
                detail=self.panel._triangulation_wait_detail(combined_warnings),
            )
            return
        if estimate is None:
            self.panel._record_unresolved_pair(
                attempt,
                status="matched_distance_invalid",
                warnings=combined_warnings,
            )
            self.panel._clear_triangulation_quality("유효한 거리 계산 대기 중")
            self.panel._set_distance_text(
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
            uses_pixhawk_attitude=(
                "live_pixhawk"
                in dict(attempt.measurement_component_sources).values()
            ),
            left_age_s=estimate.left_attitude_age_s,
            right_age_s=estimate.right_attitude_age_s,
            pair_delta_s=estimate.attitude_pair_delta_s,
        )
        self.panel._update_triangulation_quality(
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

        attitude_badge = "N/A" if attitude_timing_status == "NOT_USED" else attitude_timing_status
        display_quality = estimate.sync_quality
        if attitude_timing_status in {"STALE", "VERY_STALE"} and display_quality == "GOOD":
            display_quality = "DEGRADED"
        measurement_validity = "ESTIMATED" if estimate.warnings else "VALID"
        if measurement_validity == "ESTIMATED":
            display_quality = "ESTIMATED"
        relative_text = "relRPY=대기"
        if estimate.relative_roll_deg is not None:
            relative_text = (
                f"relRPY={estimate.relative_roll_deg:.2f}/"
                f"{estimate.relative_pitch_deg:.2f}/{estimate.relative_yaw_deg:.2f}°"
            )
        warning_text = f" | {'; '.join(estimate.warnings)}" if estimate.warnings else ""
        pair_input_unverified = any(
            bool(
                detection.timestamp_metadata.get(
                    "media_integrity_unverified",
                    False,
                )
            )
            for detection in (attempt.left_detection, attempt.right_detection)
        )
        input_integrity_text = (
            " input=UNVERIFIED" if pair_input_unverified else " input=VERIFIED"
        )
        self.panel._set_distance_text(
            f"거리: {display_distance_m:.2f} m ({method_text}{gap_text}) | bboxΔx={estimate.disparity_px:.1f}px "
            f"camΔt={estimate.time_delta_s * 1000:.0f}ms Δy={estimate.vertical_delta_px:.1f}px "
            f"baseline={display_baseline_source}:{display_baseline_m:.3f}m "
            f"ATT:{attitude_badge} {relative_text} validity={measurement_validity} "
            f"quality={display_quality}{input_integrity_text}{warning_text}",
            quality=display_quality,
        )
        appended = self.panel._append_distance_history(
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
            self.panel._set_distance_text(
                self.panel.last_distance_status,
                quality=display_quality,
                detail="그래프 미표시: 직전과 동일한 샘플이라 중복 추가를 건너뛰었습니다.",
            )

    def _show_measurement_pair(self, left: DetectionObservation, right: DetectionObservation) -> None:
        left_shown = self.panel.left_pane.show_measurement_frame_for_detection(left)
        right_shown = self.panel.right_pane.show_measurement_frame_for_detection(right)
        if not (left_shown and right_shown):
            self.panel._update_runtime_camera_stats_labels()
            return
        self.panel.selected_runtime_frame_metadata["left"] = dict(left.timestamp_metadata)
        self.panel.selected_runtime_frame_metadata["right"] = dict(right.timestamp_metadata)
        result = getattr(self.panel, "_coordinated_pairing_result", None)
        result_pair = getattr(result, "pair", None)
        timing = getattr(result, "timing", None)
        if (
            isinstance(timing, PairTimingDecision)
            and result_pair is not None
            and result_pair.left is left
            and result_pair.right is right
        ):
            self.panel.selected_runtime_frame_pair_delta_s = (
                timing.abs_delta_ns * 1.0e-9
            )
            self.panel.selected_runtime_frame_pair_basis = timing.effective_basis
            self.panel.selected_runtime_frame_pair_requested_basis = timing.requested_basis
            self.panel.selected_runtime_frame_pair_fallback_reason = (
                timing.fallback_reason
            )
        else:
            left_timestamp_s = self.panel._yolo_match_timestamp_s(left)
            right_timestamp_s = self.panel._yolo_match_timestamp_s(right)
            self.panel.selected_runtime_frame_pair_delta_s = (
                None
                if left_timestamp_s is None or right_timestamp_s is None
                else abs(left_timestamp_s - right_timestamp_s)
            )
            self.panel.selected_runtime_frame_pair_basis = (
                self.panel._effective_yolo_time_basis()
            )
            self.panel.selected_runtime_frame_pair_requested_basis = (
                self.panel._yolo_time_basis()
            )
            self.panel.selected_runtime_frame_pair_fallback_reason = ""
        _, self.panel.selected_runtime_frame_pair_camera_rtc_delta_s = (
            self.panel._selected_pair_sync_summary(left, right)
        )
        self.panel._update_runtime_camera_stats_labels()

    def _clear_measurement_pair_display(self) -> None:
        self.panel.left_pane.clear_measurement_pair_display()
        self.panel.right_pane.clear_measurement_pair_display()
        self.panel._clear_selected_runtime_measurement_pair(update_labels=True)

    def _clear_selected_runtime_measurement_pair(self, *, update_labels: bool = True) -> None:
        self.panel.selected_runtime_frame_metadata["left"] = {}
        self.panel.selected_runtime_frame_metadata["right"] = {}
        self.panel.selected_runtime_frame_pair_delta_s = None
        self.panel.selected_runtime_frame_pair_basis = ""
        self.panel.selected_runtime_frame_pair_camera_rtc_delta_s = None
        self.panel.selected_runtime_frame_pair_requested_basis = ""
        self.panel.selected_runtime_frame_pair_fallback_reason = ""
        if update_labels:
            self.panel._update_runtime_camera_stats_labels()

    def _detection_wait_detail(self) -> str:
        left_count = len(self.panel.detection_buffers["left"])
        right_count = len(self.panel.detection_buffers["right"])
        left_stats = self.panel.camera_stream_stats.get("left") or {}
        right_stats = self.panel.camera_stream_stats.get("right") or {}
        left_yolo_hz = float(left_stats.get("inferred_hz", 0.0))
        right_yolo_hz = float(right_stats.get("inferred_hz", 0.0))
        rejection_reason = str(
            self.panel._last_correspondence_rejection_reason or ""
        )
        if rejection_reason.startswith(("APP_", "RTCP_", "CLOCK_", "FRAME_")):
            result = getattr(self.panel, "_coordinated_pairing_result", None)
            primary_reason = str(
                getattr(result, "primary_rejection_reason", "") or ""
            )
            primary_detail = (
                f", preferred={primary_reason}" if primary_reason else ""
            )
            return (
                "그래프 미표시: 선택 timing과 app-receive fallback에서 유효한 pair를 "
                f"만들지 못했습니다 (effective={rejection_reason}{primary_detail})."
            )
        if rejection_reason and not rejection_reason.startswith("missing_"):
            return (
                "그래프 미표시: correspondence 검증 실패 "
                f"({rejection_reason})."
            )
        if left_count == 0 or right_count == 0:
            if left_yolo_hz <= 0.0 or right_yolo_hz <= 0.0:
                return (
                    "그래프 미표시: YOLO 추론 대기 중 "
                    f"(left={left_yolo_hz:.1f} Hz, right={right_yolo_hz:.1f} Hz)."
                )
            return f"그래프 미표시: bbox 검출 대기 중 (left={left_count}, right={right_count})."
        return (
            "그래프 미표시: 좌/우 bbox timestamp가 `최대 Δt` 안에서 아직 맞지 않습니다 "
            f"(left={left_count}, right={right_count})."
            if self.panel._yolo_time_delta_gate_enabled()
            else f"그래프 미표시: 좌/우 bbox 매칭 후보 대기 중 (left={left_count}, right={right_count})."
        )

    def _detection_wait_status(self) -> str:
        rejection_reason = str(
            self.panel._last_correspondence_rejection_reason or ""
        )
        if rejection_reason.startswith(("APP_", "RTCP_", "CLOCK_", "FRAME_")):
            return f"거리: timing pair 대기 ({rejection_reason})"
        if rejection_reason and not rejection_reason.startswith("missing_"):
            return f"거리: 검출 대응 거부 ({rejection_reason})"
        if self.panel._yolo_time_delta_gate_enabled() or self.panel._yolo_time_basis() != "detection":
            return "거리: timestamp가 맞는 좌/우 검출 대기 중"
        return "거리: 좌/우 검출 쌍 대기 중"

    def _stereo_geometry_wait_detail(self, pair) -> str:
        left = pair.left
        right = pair.right
        left_center = getattr(left, "center", (0.0, 0.0))
        right_center = getattr(right, "center", (0.0, 0.0))
        disparity_px = abs(float(left_center[0]) - float(right_center[0]))
        timing = self._pair_timing_decision(pair)
        time_delta_ms = timing.abs_delta_ns * 1.0e-6
        time_limit_ms = self.panel._yolo_pair_max_delta_s() * 1000.0
        if not isfinite(disparity_px) or disparity_px <= 1e-6:
            return (
                "그래프 미표시: 좌/우 bbox 중심의 x 차이가 0에 가까워 disparity를 만들 수 없습니다. "
                f"left_x={float(left_center[0]):.1f}px, right_x={float(right_center[0]):.1f}px."
            )
        if time_delta_ms > time_limit_ms:
            return (
                "그래프 미표시: 좌/우 bbox timestamp 차이가 `최대 Δt`보다 큽니다. "
                f"camΔt={time_delta_ms:.0f}ms, limit={time_limit_ms:.0f}ms."
            )
        return (
            "그래프 미표시: 좌/우 bbox는 선택됐지만 거리 계산용 stereo match가 생성되지 않았습니다. "
            f"disparity={disparity_px:.1f}px, camΔt={time_delta_ms:.0f}ms."
        )

    def _triangulation_wait_detail(self, warnings: tuple[str, ...]) -> str:
        warning_text = "; ".join(warnings)
        if any("waiting for intrinsics" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 내부 파라미터가 아직 적용되지 않아 3D ray triangulation을 "
                "계산할 수 없습니다. `줌 내부 파라미터`에서 intrinsics JSON을 불러온 뒤 `계산에 적용`을 누르세요. "
                f"{warning_text}"
            )
        if any("waiting for Pixhawk R" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 Pixhawk 자세 샘플이 아직 없어 3D ray 방향을 만들 수 없습니다. "
                f"{warning_text}"
            )
        if any("waiting for checkerboard stereo extrinsics" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 적용된 체커보드 calibration에 stereo 외부 파라미터가 없습니다. "
                "좌/우 체커보드 쌍을 최소 3개 이상 보정한 뒤 결과를 계산에 적용하세요. "
                f"{warning_text}"
            )
        if any("waiting for RTK T" in warning for warning in warnings):
            return (
                "그래프 미표시: bbox 매칭은 됐지만 좌/우 카메라 translation을 만들 RTK/GPS baseline이 아직 없습니다. "
                f"{warning_text}"
            )
        if any("triangulation error" in warning for warning in warnings):
            return f"그래프 미표시: triangulation 계산 중 오류가 발생했습니다. {warning_text}"
        if warning_text:
            return f"그래프 미표시: 3D ray triangulation 결과가 아직 없습니다. {warning_text}"
        return "그래프 미표시: bbox 매칭은 됐지만 3D ray triangulation 결과가 아직 없습니다."

    def _format_distance_label_text(self, text: str, detail: str | None = None) -> str:
        if not detail:
            return text
        return (
            "<div style='line-height: 1.1;'>"
            f"{html_lib.escape(text)}"
            "<br>"
            f"<span style='font-size: 9px; font-weight: 400;'>{html_lib.escape(detail)}</span>"
            "</div>"
        )

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
        return (
            f"Attitude timing [{status}] | {baseline_text} | "
            f"Left age {self.panel._format_attitude_timing_age(left_age_s)} | "
            f"Right age {self.panel._format_attitude_timing_age(right_age_s)} | "
            f"Pair Δt {self.panel._format_attitude_timing_age(pair_delta_s)} | "
            "display-only; measurement continues"
        )

    def _clear_triangulation_quality(self, reason: str) -> None:
        if not hasattr(self.panel, "triangulation_quality_status_label"):
            return
        self.panel.triangulation_quality_status_label.setText(reason)
        self.panel.triangulation_quality_gap_label.setText("gap: --")
        self.panel.triangulation_quality_distance_label.setText("distance: --")
        self.panel.triangulation_quality_mode_label.setText("mode: --")
        self.panel._set_health_label(self.panel.triangulation_quality_attitude_label, "Attitude timing: --", "idle")
        self.panel.triangulation_quality_raw_point_label.setText("Raw baseline_center_frd point: --")
        self.panel.triangulation_quality_plot.set_quality(None)

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
        if not hasattr(self.panel, "triangulation_quality_status_label"):
            return
        self.panel.triangulation_quality_status_label.setText(
            "표시 전용: 현재 좌/우 bbox ray의 최단거리 gap과 3D midpoint를 보여줍니다."
        )
        self.panel.triangulation_quality_gap_label.setText(f"gap: {triangulated.closest_gap_m:.3f} m")
        self.panel.triangulation_quality_distance_label.setText(f"distance: {display_distance_m:.3f} m")
        self.panel.triangulation_quality_mode_label.setText(f"mode: {method_text}")
        self.panel._set_health_label(
            self.panel.triangulation_quality_attitude_label,
            self.panel._format_attitude_timing_text(
                status=attitude_timing_status,
                left_age_s=left_attitude_age_s,
                right_age_s=right_attitude_age_s,
                pair_delta_s=attitude_pair_delta_s,
                baseline_source=display_baseline_source,
                baseline_m=display_baseline_m,
            ),
            self.panel._attitude_timing_health_level(attitude_timing_status),
        )
        self.panel.triangulation_quality_raw_point_label.setText(
            "Raw baseline_center_frd point [F/R/D]: "
            f"{self.panel._format_quality_point(public_point_m)} | origin=baseline midpoint "
            "| source left_camera_rdf [R/D/F]: "
            f"{self.panel._format_quality_point(source_point_left_rdf_m)}"
        )
        self.panel.triangulation_quality_plot.set_quality(
            {
                "closest_gap_m": triangulated.closest_gap_m,
                "distance_m": display_distance_m,
                "method": method_text,
                "scene": triangulation_3d_scene_points(triangulated, baseline_m=display_baseline_m),
            }
        )

    def _set_distance_text(self, text: str, *, quality: str = "unknown", detail: str | None = None) -> None:
        self.panel.last_distance_status = text
        self.panel.last_distance_quality = quality
        if hasattr(self.panel, "distance_plot"):
            self.panel.distance_plot.set_empty_message(detail or text)
        if detail:
            self.panel.distance_label.setTextFormat(QtCore.Qt.RichText)
        else:
            self.panel.distance_label.setTextFormat(QtCore.Qt.PlainText)
        self.panel.distance_label.setText(self.panel._format_distance_label_text(text, detail))
        style = self.panel._distance_style(quality)
        self.panel.distance_label.setStyleSheet(
            "font-size: 14px; font-weight: 700; padding: 8px 10px; "
            f"border: 1px solid {PANEL_BORDER}; border-radius: 0px; " + style
        )
        self.panel._refresh_operation_header()

    def _distance_style(self, quality: str) -> str:
        if quality == "GOOD":
            return f"background: {PANEL_ALT_BG}; color: {SUCCESS_TEXT}; border-color: {SUCCESS};"
        if quality in {"DEGRADED", "WARNING", "WARN", "ESTIMATED"}:
            return f"background: {PANEL_ALT_BG}; color: {WARNING_TEXT}; border-color: {WARNING};"
        if quality in {"BAD", "ERROR"}:
            return f"background: {PANEL_ALT_BG}; color: {ERROR_TEXT}; border-color: {ERROR};"
        return f"background: {PANEL_ALT_BG}; color: {TEXT_MUTED}; border-color: {PANEL_BORDER};"
