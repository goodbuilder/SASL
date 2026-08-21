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
)
from Workstation.workstation_core.measurement_controller import calculate_measurement_geometry
from Workstation.workstation_core.measurement_widgets import triangulation_3d_scene_points
from Workstation.workstation_core.rtk_geometry import (
    GpsPairReadiness,
    GpsPositionState,
    RtkBaselineEstimate,
    estimate_rtk_baseline,
    ned_delta_m,
    validate_gps_pair_for_baseline,
)
from Workstation.workstation_core.stereo import DetectionObservation
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

    def _measurement_method_state(self) -> None:
        panel = self.panel
        measurement_id = str(getattr(panel, "_measurement_id", "") or "")
        package = getattr(panel, "_measurement_package", None)
        config_token = (measurement_id, id(package) if package is not None else 0)
        if getattr(panel, "_measurement_method_config_token", None) != config_token:
            metadata: dict[str, object] = {}
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
            global_preferred = normalize_measurement_method_id(
                panel._workstation_setting(
                    "measurement_preferred_method",
                    getattr(panel.settings, "measurement_preferred_method", PIXHAWK_RTK_LIVE),
                )
            )
            global_order = normalize_measurement_method_fallback_order(
                panel._workstation_setting(
                    "measurement_method_fallback_order",
                    getattr(
                        panel.settings,
                        "measurement_method_fallback_order",
                        DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
                    ),
                )
            )
            panel._measurement_preferred_method_id = normalize_measurement_method_id(
                metadata.get("measurement_preferred_method", global_preferred)
            )
            panel._measurement_active_fallback_order = normalize_measurement_method_fallback_order(
                metadata.get("measurement_method_fallback_order", global_order)
            )
            panel._measurement_method_fill_policy = normalize_measurement_method_fill_policy(
                metadata.get("measurement_method_fill_policy")
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
        return {
            "measurement_method_config_schema": "measurement-method-config-v1",
            "measurement_preferred_method": preferred,
            "measurement_method_fallback_order": list(order),
            "measurement_method_fill_policy": default_measurement_method_fill_policy(),
        }

    def _measurement_fill_policy(self) -> dict[str, dict[str, str]]:
        self._measurement_method_state()
        return normalize_measurement_method_fill_policy(
            getattr(self.panel, "_measurement_method_fill_policy", None)
        )

    def _measurement_fill_source(self, method_id: str, side: str, dof: str) -> str:
        return self._measurement_fill_policy()[method_id][f"{side}.{dof}"]

    def _measurement_live_readiness(self) -> tuple[bool, bool, tuple[str, ...]]:
        """Return fresh Pixhawk and usable GNSS readiness for hybrid fills."""

        now_s = time.monotonic()
        extrinsics = getattr(self.panel, "latest_extrinsics", {}) or {}
        attitude_reasons: list[str] = []
        for side in ("left", "right"):
            sample = extrinsics.get(side) if isinstance(extrinsics, dict) else None
            if sample is None:
                attitude_reasons.append(f"{side} Pixhawk attitude missing")
                continue
            timestamp_s = getattr(sample, "timestamp_s", None)
            try:
                age_s = now_s - float(timestamp_s)
            except (TypeError, ValueError):
                age_s = float("nan")
            if not math.isfinite(age_s) or age_s < 0.0 or age_s > 2.0:
                attitude_reasons.append(f"{side} Pixhawk attitude stale")
        pixhawk_ready = not attitude_reasons
        baseline = getattr(self.panel, "latest_rtk_baseline", None)
        gnss_ready = False
        gnss_reason = "live GNSS baseline missing"
        if baseline is not None:
            usable, reason = self.panel._gps_baseline_usable_for_translation(baseline)
            gnss_ready = bool(usable)
            gnss_reason = str(reason or "live GNSS baseline unusable")
        if gnss_ready and bool(getattr(self.panel, "gnss_operation_gate_active", False)):
            manager = getattr(self.panel, "gnss_operation_manager", None)
            readiness = None if manager is None else manager.readiness()
            config = None if manager is None else manager.config
            if (
                config is not None
                and bool(getattr(config, "readiness_gate_enabled", False))
                and (readiness is None or not readiness.ready)
            ):
                gnss_ready = False
                gnss_reason = (
                    "GNSS readiness snapshot missing"
                    if readiness is None
                    else "; ".join(readiness.reasons) or "GNSS readiness blocked"
                )
        reasons = tuple(attitude_reasons + ([] if gnss_ready else [gnss_reason]))
        return pixhawk_ready, gnss_ready, reasons

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
        if getattr(self.panel, "active_intrinsics", None) is None:
            reasons.append("exact operating K,D missing")
        applicability: object = None
        try:
            snapshot = owner._intrinsics_snapshot(use_frozen=False)
            applicability = snapshot["applicability"]
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError):
            reasons.append("exact operating K,D applicability unavailable")
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

    def _measurement_method_statuses(self) -> tuple[MeasurementMethodStatus, ...]:
        """Project every geometry source into independent method readiness."""

        owner = self.panel._optical_pose_controller()
        _geometry_allowed, current_profile_hash, intrinsics_reasons = (
            self._operating_intrinsics_readiness(owner)
        )
        pixhawk_ready, gnss_ready, live_reasons = self._measurement_live_readiness()
        statuses: list[MeasurementMethodStatus] = []

        pixhawk_reasons = list(intrinsics_reasons)
        if not pixhawk_ready:
            pixhawk_reasons.extend(reason for reason in live_reasons if "Pixhawk" in reason)
        if not gnss_ready:
            pixhawk_reasons.extend(reason for reason in live_reasons if "Pixhawk" not in reason)
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
            )
        )

        checkerboard_profile = getattr(self.panel, "checkerboard_extrinsics_profile", None)
        checkerboard_reasons = list(intrinsics_reasons)
        if checkerboard_profile is None:
            checkerboard_reasons.append("checkerboard stereo R,t missing")
        statuses.append(
            MeasurementMethodStatus(
                method_id=CHECKERBOARD_STEREO,
                instance_id=(
                    "profile:checkerboard:missing"
                    if checkerboard_profile is None
                    else f"profile:{checkerboard_profile.profile_id or checkerboard_profile.content_sha256}"
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

        for candidate in getattr(owner, "_candidates", {}).values():
            method_id = measurement_method_id_for_candidate(candidate.method)
            if method_id is None:
                continue
            observed_by_side = {
                side: self._candidate_observed_dofs(candidate, side)
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
                reasons.extend(reason for reason in live_reasons if "Pixhawk" in reason)
            if requires_live_gnss and not gnss_ready:
                reasons.append("usable live GNSS required for missing translation axes")
                reasons.extend(reason for reason in live_reasons if "Pixhawk" not in reason)
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
                    instance_id=f"candidate:{candidate.candidate_id}",
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
                    warnings=tuple(candidate.warnings),
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
        if loaded_profile is not None and loaded_method_id is not None:
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
                if not expected_profile_hash:
                    reasons.append("loaded optical profile operating K,D SHA-256 missing")
                elif expected_profile_hash != current_profile_hash:
                    reasons.append("loaded optical profile K,D differs from active K,D")
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
        pixhawk_ready, gnss_ready, _reasons = self._measurement_live_readiness()
        locked = self._measurement_fill_policy_locked()
        rows: dict[str, dict[str, object]] = {}
        atomic_method = status.method_id in {PIXHAWK_RTK_LIVE, CHECKERBOARD_STEREO} or bool(
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

        return {
            "uses_live_pixhawk": MEASUREMENT_FILL_LIVE_PIXHAWK in sources,
            "uses_live_gnss": MEASUREMENT_FILL_LIVE_GNSS in sources,
            "live_attitude_rpy_deg": attitude_rows,
            "live_baseline_ned_m": baseline_row,
            "candidate_pose_virtual_frd_to_world_ned": candidate_rows,
            "stereo_extrinsics_opencv_rdf": extrinsics_row,
        }

    def workstation_measurement_methods_snapshot(self) -> dict[str, object]:
        self._measurement_method_state()
        selected = self._select_measurement_method_for_attempt()
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
        for method_id in MEASUREMENT_METHOD_IDS:
            capability = capability_rows.get(method_id)
            if capability is not None:
                method_catalog.append(capability)
                continue
            method_catalog.append(
                {
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
            )
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
        current = getattr(self.panel, "active_intrinsics", None)
        if current is None:
            return None
        intrinsics_only = without_stereo_extrinsics(current)
        if status.method_id == PIXHAWK_RTK_LIVE:
            return intrinsics_only
        if status.method_id == CHECKERBOARD_STEREO:
            profile = getattr(self.panel, "checkerboard_extrinsics_profile", None)
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
            "left_gps": gps_payload(left_gps),
            "right_gps": gps_payload(right_gps),
        }
        return hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()

    def _record_no_ready_measurement_pair(self, pair) -> None:
        timestamp_basis = self.panel._effective_yolo_time_basis()
        left_pairing_timestamp_s = self.panel._yolo_match_timestamp_s(pair.left)
        right_pairing_timestamp_s = self.panel._yolo_match_timestamp_s(pair.right)
        if left_pairing_timestamp_s is None or right_pairing_timestamp_s is None:
            return
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
        )
        self.panel._record_blocked_measurement_pair(blocked)

    def _update_detection(self, camera: str, detection: DetectionObservation | None) -> None:
        if camera == "left":
            self.panel.latest_left = detection
        elif camera == "right":
            self.panel.latest_right = detection
        if detection is not None:
            self.panel.detection_buffers[camera].add(detection)
        self.panel._refresh_distance()
        self.panel._refresh_optical_alignment_view()

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
        left = self.panel.latest_gps.get("left")
        right = self.panel.latest_gps.get("right")

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
        return f"RTK Fixed: {fixed_text} | GPS baseline 필수, {usability} | {side_text('P1', left)} | {side_text('P2', right)}"

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
        observed_dx_px = float(left_center_x) - float(right_center_x)
        observed_dy_px = float(left_center_y) - float(right_center_y)
        expected_dy_px, expected_dy_source, expected_dy_level = self.panel._optical_alignment_expected_dy_px()
        dy_residual_px = None if expected_dy_px is None else observed_dy_px - expected_dy_px
        mean_fy_px, fy_source = self.panel._optical_alignment_mean_focal_px("y")
        mean_fx_px, fx_source = self.panel._optical_alignment_mean_focal_px("x")
        vertical_error_mrad = None if dy_residual_px is None or mean_fy_px is None else dy_residual_px / mean_fy_px * 1000.0
        vertical_level = self.panel._optical_alignment_level_from_mrad(vertical_error_mrad)

        left_timestamp_s = self.panel._yolo_match_timestamp_s(pair.left)
        right_timestamp_s = self.panel._yolo_match_timestamp_s(pair.right)
        camera_delta_s = (
            None if left_timestamp_s is None or right_timestamp_s is None else abs(left_timestamp_s - right_timestamp_s)
        )
        try:
            attempt = self.panel._build_measurement_attempt(pair)
            geometry_result = calculate_measurement_geometry(attempt)
        except (TypeError, ValueError) as exc:
            attempt = None
            geometry_result = None
            geometry_error = str(exc)
        else:
            geometry_error = ""

        baseline_m = 0.0 if attempt is None else attempt.baseline_m
        baseline_source = "waiting" if attempt is None else attempt.baseline_source
        target_distance_m = self.panel.optical_target_distance_spin.value()
        expected_disparity_px = None
        yaw_residual_px = None
        yaw_error_mrad = None
        if mean_fx_px is not None and baseline_m > 0.0 and target_distance_m > 0.0:
            expected_disparity_mag_px = mean_fx_px * baseline_m / target_distance_m
            expected_disparity_px = expected_disparity_mag_px if observed_dx_px >= 0.0 else -expected_disparity_mag_px
            yaw_residual_px = observed_dx_px - expected_disparity_px
            yaw_error_mrad = yaw_residual_px / mean_fx_px * 1000.0
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

    def _build_measurement_attempt(self, pair) -> MeasurementAttempt:
        """Freeze pair inputs first, then resolve one immutable method geometry."""

        method_status = self._select_measurement_method_for_attempt()
        if method_status is None:
            raise ValueError("no ready measurement method")
        method_id = method_status.method_id
        triangulation_source = "pixhawk" if method_id == PIXHAWK_RTK_LIVE else "checkerboard"
        geometry_kind = "live_pixhawk_mount" if method_id == PIXHAWK_RTK_LIVE else "relative_pose"
        timestamp_basis = self.panel._effective_yolo_time_basis()
        left_pairing_timestamp_s = self.panel._yolo_match_timestamp_s(pair.left)
        right_pairing_timestamp_s = self.panel._yolo_match_timestamp_s(pair.right)
        if left_pairing_timestamp_s is None or right_pairing_timestamp_s is None:
            raise ValueError("matched detection pair has no timestamp on its selected basis")

        created_monotonic_s = time.monotonic()
        detection_midpoint_s = (float(pair.left.timestamp_s) + float(pair.right.timestamp_s)) / 2.0
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

        left_nearest = self.panel.attitude_buffers["left"].nearest(float(pair.left.timestamp_s))
        right_nearest = self.panel.attitude_buffers["right"].nearest(float(pair.right.timestamp_s))
        left_attitude = None if left_nearest is None else AttitudeSnapshot.from_state(left_nearest.sample)
        right_attitude = None if right_nearest is None else AttitudeSnapshot.from_state(right_nearest.sample)
        left_attitude_age_s = None if left_nearest is None else left_nearest.age_s
        right_attitude_age_s = None if right_nearest is None else right_nearest.age_s
        attitude_pair_delta_s = (
            None
            if left_attitude is None or right_attitude is None
            else abs(left_attitude.timestamp_s - right_attitude.timestamp_s)
        )
        relative = (
            None
            if left_attitude is None or right_attitude is None
            else relative_attitude(left_attitude.quaternion_wxyz, right_attitude.quaternion_wxyz)
        )

        selected_left_gps, selected_right_gps = self.panel._selected_baseline_gps_states()
        left_gps = None if selected_left_gps is None else GpsSnapshot.from_state(selected_left_gps)
        right_gps = None if selected_right_gps is None else GpsSnapshot.from_state(selected_right_gps)

        attitude_for_geometry_left = (
            left_attitude
            if left_attitude_age_s is not None and left_attitude_age_s <= 2.0
            else None
        )
        attitude_for_geometry_right = (
            right_attitude
            if right_attitude_age_s is not None and right_attitude_age_s <= 2.0
            else None
        )
        measurement_result = self._measurement_intrinsics_result_for_method(
            method_status,
            left_attitude=attitude_for_geometry_left,
            right_attitude=attitude_for_geometry_right,
            left_gps=left_gps,
            right_gps=right_gps,
        )
        intrinsics = None
        if measurement_result is not None:
            profile_path = self.panel._active_intrinsics_profile_path or (
                f"memory:active-calibration:zoom-{self.panel.zoom_level_spin.value():02d}"
            )
            intrinsics = StereoIntrinsicsSnapshot.from_result(
                measurement_result,
                active_zoom_level=self.panel.zoom_level_spin.value(),
                selected_profile_path=profile_path,
                selected_extrinsics_profile_path=method_status.instance_id,
                intrinsics_source_zoom_level=getattr(
                    self.panel, "_active_intrinsics_source_zoom_level", None
                ),
                extrinsics_source_zoom_level=getattr(
                    self.panel, "_active_extrinsics_source_zoom_level", None
                ),
                selection_mode=f"measurement_method/{method_id}",
            )

        warnings: tuple[str, ...] = tuple(method_status.warnings)
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
                        None if left_attitude is None else left_attitude.quaternion_wxyz
                    ),
                    selected_right_quaternion_wxyz=(
                        None if right_attitude is None else right_attitude.quaternion_wxyz
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
            if key == self.panel._last_measurement_attempt_key
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
        )
        candidate_attempt_id = self.panel._measurement_attempt_counter + 1
        attempt = MeasurementAttempt(
            attempt_id=candidate_attempt_id,
            attempt_session_id=self.panel._measurement_attempt_generation_id,
            detection_pair_key=key,
            created_monotonic_s=created_monotonic_s,
            created_wall_s=time.time(),
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
        )
        if cached_attempt is not None:
            previous_missing = set(cached_attempt.missing_geometry_inputs)
            candidate_missing = set(attempt.missing_geometry_inputs)
            if not candidate_missing < previous_missing:
                return cached_attempt
        self.panel._measurement_attempt_counter = candidate_attempt_id
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
        right_ext = self.panel.latest_extrinsics.get("right")
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        return camera_translation_right_in_left_camera(
            baseline_right_in_left_world_m=world_vector,
            left_body_to_world_q=left_ext.q,
            right_body_to_world_q=right_ext.q if right_ext is not None else left_ext.q,
            scale_to_m=None,
        )

    def _alignment_view_camera_translation_right_in_left_m(
        self, baseline: RtkBaselineEstimate
    ) -> tuple[float, float, float] | None:
        left_ext = self.panel.latest_extrinsics.get("left")
        if left_ext is None:
            return None
        right_ext = self.panel.latest_extrinsics.get("right")
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        scale_to_m = self.panel.baseline_spin.value() if hasattr(self.panel, "baseline_spin") else None
        return camera_translation_right_in_left_camera(
            baseline_right_in_left_world_m=world_vector,
            left_body_to_world_q=left_ext.q,
            right_body_to_world_q=right_ext.q if right_ext is not None else left_ext.q,
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
        device_ids = {
            side: str(getattr(sample, "topic", side))
            for side, sample in self.panel.latest_gps.items()
            if side in {"left", "right", "drone"}
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
        left_pixel = pair.left.center
        right_pixel = pair.right.center
        rows: list[dict[str, object]] = []

        def add_row(label: str, attempt: object | None, *, provenance_note: str = "") -> None:
            suffix = f" · {provenance_note}" if provenance_note else ""
            if attempt is None:
                rows.append(
                    {
                        "method": label,
                        "forward_m": None,
                        "right_m": None,
                        "down_m": None,
                        "range_m": None,
                        "status": f"waiting for inputs{suffix}",
                    }
                )
                return
            result = attempt.result
            if result is None:
                rows.append(
                    {
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
                    right_body_to_world_q=right_ext.q if right_ext is not None else left_ext.q,
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
        add_row("Pixhawk + RTK (live)", pixhawk_attempt)

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
        add_row("Checkerboard", checkerboard_attempt)

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
        )

        # 4) Every unapplied PnP candidate (latest solve per method), so an
        # operator can compare Horizon/Mountain/Drone-yaw/Combined solves
        # before deciding whether to apply one.
        candidate_poses = self.panel.workstation_optical_pose_candidate_poses()
        for candidate_row in candidate_poses:
            stereo = stereo_pose_from_absolute(
                candidate_row["left_pose_ned"],
                candidate_row["right_pose_ned"],
            )
            attempt = relative_pose_attempt(
                [list(row) for row in stereo.rotation_right_from_left_rdf],
                list(stereo.translation_right_from_left_rdf_m),
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
            add_row(f"PnP candidate: {candidate_row['method']}", attempt, provenance_note=note)

        return tuple(rows)

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
        selected_method_status = self._select_measurement_method_for_attempt()
        if selected_method_status is None:
            self._record_no_ready_measurement_pair(pair)
            self.panel._clear_triangulation_quality("준비된 측정 방식 없음")
            self.panel._set_distance_text(
                "거리: 준비된 측정 방식 없음",
                quality="WARNING",
                detail="해당 matched pair를 geometry 미준비 상태로 기록했습니다.",
            )
            return
        gnss_measurement_block_reason: str | None = None
        gnss_measurement_block_warnings: tuple[str, ...] = ()
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
                # A failed readiness check invalidates this measurement result,
                # but must not discard the matched pair while a recording is
                # active.  Build the immutable attempt below and persist it as
                # an unresolved sample with the GNSS reason attached.
                gnss_measurement_block_reason = "readiness"
                gnss_measurement_block_warnings = tuple(reasons) or (reason_text,)
            if readiness is not None and gnss_measurement_block_reason is None:
                try:
                    contract = self.panel._capture_measurement_gnss_contract(readiness)
                    if self.panel.gnss_operation_manager.state == GnssOperationState.READY:
                        self.panel.gnss_operation_manager.begin_measurement(contract, readiness)
                    elif self.panel.gnss_operation_manager.state == GnssOperationState.MEASURING:
                        if not self.panel.gnss_operation_manager.validate_measurement_contract(contract, readiness):
                            raise RuntimeError("GNSS measurement contract changed")
                except (RuntimeError, ValueError) as exc:
                    # Keep the pair for recording even when its GNSS contract
                    # cannot be accepted for a valid distance result.
                    gnss_measurement_block_reason = "contract"
                    gnss_measurement_block_warnings = (str(exc),)
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

        if gnss_measurement_block_reason is not None:
            warnings = tuple(dict.fromkeys((*attempt.warnings, *gnss_measurement_block_warnings)))
            self.panel._record_unresolved_pair(
                attempt,
                status=f"matched_gnss_{gnss_measurement_block_reason}_blocked",
                warnings=warnings,
            )
            self.panel._clear_triangulation_quality("GNSS 운용 준비 조건 미충족")
            self.panel._set_distance_text(
                "거리: GNSS 운용 준비 조건 미충족 | " + "; ".join(warnings),
                quality="WARNING",
                detail="그래프 미표시: 해당 pair를 무효 측정으로 기록했습니다.",
            )
            return

        baseline_m = attempt.baseline_m
        baseline_source = attempt.baseline_source
        if baseline_source in {"rtk-blocked", "gps-rtk-blocked"}:
            self.panel._record_unresolved_pair(
                attempt,
                status="matched_baseline_blocked",
                warnings=attempt.warnings,
            )
            self.panel._clear_triangulation_quality("GPS baseline 조건 미충족")
            self.panel._set_distance_text(
                "거리: GPS baseline 조건 미충족 | " + "; ".join(attempt.warnings),
                quality="WARNING",
                detail="그래프 미표시: GPS baseline 조건을 만족하지 못했습니다.",
            )
            return
        triangulation_source = attempt.triangulation_source
        geometry_result = calculate_measurement_geometry(attempt)
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
        relative_text = "relRPY=대기"
        if estimate.relative_roll_deg is not None:
            relative_text = (
                f"relRPY={estimate.relative_roll_deg:.2f}/"
                f"{estimate.relative_pitch_deg:.2f}/{estimate.relative_yaw_deg:.2f}°"
            )
        warning_text = f" | {'; '.join(estimate.warnings)}" if estimate.warnings else ""
        self.panel._set_distance_text(
            f"거리: {display_distance_m:.2f} m ({method_text}{gap_text}) | bboxΔx={estimate.disparity_px:.1f}px "
            f"camΔt={estimate.time_delta_s * 1000:.0f}ms Δy={estimate.vertical_delta_px:.1f}px "
            f"baseline={display_baseline_source}:{display_baseline_m:.3f}m "
            f"ATT:{attitude_badge} {relative_text} quality={display_quality}{warning_text}",
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
        left_timestamp_s = self.panel._yolo_match_timestamp_s(left)
        right_timestamp_s = self.panel._yolo_match_timestamp_s(right)
        self.panel.selected_runtime_frame_pair_delta_s = (
            None if left_timestamp_s is None or right_timestamp_s is None else abs(left_timestamp_s - right_timestamp_s)
        )
        (
            self.panel.selected_runtime_frame_pair_basis,
            self.panel.selected_runtime_frame_pair_camera_rtc_delta_s,
        ) = self.panel._selected_pair_sync_summary(left, right)
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
        if update_labels:
            self.panel._update_runtime_camera_stats_labels()

    def _detection_wait_detail(self) -> str:
        left_count = len(self.panel.detection_buffers["left"])
        right_count = len(self.panel.detection_buffers["right"])
        left_stats = self.panel.camera_stream_stats.get("left") or {}
        right_stats = self.panel.camera_stream_stats.get("right") or {}
        left_yolo_hz = float(left_stats.get("inferred_hz", 0.0))
        right_yolo_hz = float(right_stats.get("inferred_hz", 0.0))
        if self.panel._last_correspondence_rejection_reason and not self.panel._last_correspondence_rejection_reason.startswith(
            "missing_"
        ):
            return (
                "그래프 미표시: correspondence 검증 실패 "
                f"({self.panel._last_correspondence_rejection_reason})."
            )
        if left_count == 0 or right_count == 0:
            if left_yolo_hz <= 0.0 or right_yolo_hz <= 0.0:
                return (
                    "그래프 미표시: YOLO 추론 대기 중 "
                    f"(left={left_yolo_hz:.1f} Hz, right={right_yolo_hz:.1f} Hz)."
                )
            return f"그래프 미표시: bbox 검출 대기 중 (left={left_count}, right={right_count})."
        if self.panel._yolo_time_basis() == "camera_rtc_required":
            left_has_rtc = any(
                self.panel._camera_rtc_timestamp_s_from_metadata(detection.timestamp_metadata) is not None
                for detection in self.panel.detection_buffers["left"]
            )
            right_has_rtc = any(
                self.panel._camera_rtc_timestamp_s_from_metadata(detection.timestamp_metadata) is not None
                for detection in self.panel.detection_buffers["right"]
            )
            if not left_has_rtc or not right_has_rtc:
                return (
                    "그래프 미표시: RTCP/NTP 필수 모드에서 RTCP/NTP timestamp가 없는 검출이 있습니다 "
                    f"(left_rtc={'OK' if left_has_rtc else 'missing'}, "
                    f"right_rtc={'OK' if right_has_rtc else 'missing'})."
                )
        return (
            "그래프 미표시: 좌/우 bbox timestamp가 `최대 Δt` 안에서 아직 맞지 않습니다 "
            f"(left={left_count}, right={right_count})."
            if self.panel._yolo_time_delta_gate_enabled()
            else f"그래프 미표시: 좌/우 bbox 매칭 후보 대기 중 (left={left_count}, right={right_count})."
        )

    def _detection_wait_status(self) -> str:
        if self.panel._last_correspondence_rejection_reason and not self.panel._last_correspondence_rejection_reason.startswith(
            "missing_"
        ):
            return f"거리: 검출 대응 거부 ({self.panel._last_correspondence_rejection_reason})"
        if self.panel._yolo_time_delta_gate_enabled() or self.panel._yolo_time_basis() != "detection":
            return "거리: timestamp가 맞는 좌/우 검출 대기 중"
        return "거리: 좌/우 검출 쌍 대기 중"

    def _stereo_geometry_wait_detail(self, pair) -> str:
        left = pair.left
        right = pair.right
        left_center = getattr(left, "center", (0.0, 0.0))
        right_center = getattr(right, "center", (0.0, 0.0))
        disparity_px = abs(float(left_center[0]) - float(right_center[0]))
        left_timestamp_s = self.panel._yolo_match_timestamp_s(left)
        right_timestamp_s = self.panel._yolo_match_timestamp_s(right)
        time_delta_ms = (
            float("inf")
            if left_timestamp_s is None or right_timestamp_s is None
            else abs(left_timestamp_s - right_timestamp_s) * 1000.0
        )
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
        if quality in {"DEGRADED", "WARNING", "WARN"}:
            return f"background: {PANEL_ALT_BG}; color: {WARNING_TEXT}; border-color: {WARNING};"
        if quality in {"BAD", "ERROR"}:
            return f"background: {PANEL_ALT_BG}; color: {ERROR_TEXT}; border-color: {ERROR};"
        return f"background: {PANEL_ALT_BG}; color: {TEXT_MUTED}; border-color: {PANEL_BORDER};"
