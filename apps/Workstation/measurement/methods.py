"""Method configuration, readiness and selection.

Attitudes: body FRD -> earth NED, wxyz. GPS baselines: NED metres.
Optical R,t: X_right_RDF = R_right_from_left X_left_RDF + t_m.
Public point_m: baseline-center FRD; source_point_left_rdf_m: left optical RDF.
"""

from __future__ import annotations
import hashlib
import json
import math
import time
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from Workstation.workstation_core.calibration import (
    intrinsics_content_sha256,
    without_stereo_extrinsics,
)
from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    CLOUD_GNSS,
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
    pose_parameters_from_camera_pose,
)
from Workstation.workstation_core.operating_intrinsics import (
    NOMINAL_INTRINSICS_QUALITY,
    nominal_profile_to_stereo_result,
)
from Workstation.workstation_core.gnss_operation import ReadinessConfig
from Workstation.workstation_core.method_evaluation import MethodEvaluationBundle
from Workstation.workstation_core.settings import normalize_live_distance_time_basis
from Workstation.workstation_core.rtk_geometry import ned_delta_m
from typing import Any, Callable
from Workstation.measurement import limits
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.optical_inputs import OpticalMeasurementInputs
from Workstation.measurement.state import (
    UNSET,
    read,
    PairCaptureState,
    ResultCommitState,
    GeometryState,
    MethodSelectionState,
)


@dataclass(frozen=True)
class MethodSelectionInputs:
    _measurement_id: ValueInput
    _measurement_package: ValueInput
    _measurement_run_metadata: ValueInput
    settings: ValueInput
    _workstation_setting: ValueInput
    optical: OpticalMeasurementInputs
    active_intrinsics: ValueInput
    _active_intrinsics_profile_path: ValueInput
    checkerboard_extrinsics_candidate_profile: ValueInput
    latest_rtk_baseline: ValueInput
    gnss_operation_gate_active: ValueInput
    gnss_operation_manager: ValueInput
    checkerboard_extrinsics_profile: ValueInput
    optical_pose_extrinsics_profile: ValueInput
    _measurement_package_state: ValueInput
    _measurement_package_detection_started: ValueInput
    append_workstation_run_event: ValueInput
    latest_extrinsics: ValueInput


@dataclass(frozen=True)
class MethodSelectionActions:
    _configured_time_basis: Callable[..., Any]
    _cached_measurement_attitude_states: Callable[..., Any]
    _cached_measurement_gps_states: Callable[..., Any]
    panel__gps_baseline_usable_for_translation: ValueInput
    _gnss_readiness_config: Callable[..., Any]
    panel__measurement_baseline_yaw_reverse: ValueInput
    _baseline_yaw_selection_preview: Callable[..., Any]
    _geometry_evidence_sha256: Callable[..., Any]


class MethodSelectionService:
    """Method configuration, readiness and selection."""

    def __init__(
        self,
        *,
        state: MethodSelectionState,
        inputs: MethodSelectionInputs,
        actions: MethodSelectionActions,
        capture_state: PairCaptureState,
        commit_state: ResultCommitState,
        geometry_state: GeometryState,
    ) -> None:
        self.state = state
        self.inputs = inputs
        self.actions = actions
        self.capture_state = capture_state
        self.commit_state = commit_state
        self.geometry_state = geometry_state

    def measurement_calibration_snapshot(self) -> dict[str, object]:
        """K,D actually frozen for the Measurement, for the workbook's 03_Calibration."""
        result = read(self.state._pending_measurement_frozen_intrinsics_result, None)
        if result is None:
            return {}

        def camera(intrinsics: object) -> dict[str, object]:
            return {
                "width_px": int(getattr(intrinsics, "width", 0)),
                "height_px": int(getattr(intrinsics, "height", 0)),
                "camera_matrix": [
                    [float(value) for value in row]
                    for row in getattr(intrinsics, "camera_matrix", ())
                ],
                "distortion_coefficients": [
                    float(value)
                    for value in getattr(intrinsics, "distortion_coefficients", ())
                ],
                "reprojection_error_px": float(
                    getattr(intrinsics, "reprojection_error_px", 0.0) or 0.0
                ),
                "image_count": int(getattr(intrinsics, "image_count", 0) or 0),
            }

        try:
            profile_sha256 = intrinsics_content_sha256(result)
        except Exception:
            profile_sha256 = ""
        return {
            "profile_sha256": profile_sha256,
            "source_path": str(
                read(self.state._pending_measurement_frozen_intrinsics_path, "") or ""
            ),
            "left": camera(result.left),
            "right": camera(result.right),
        }

    def _measurement_method_state(self) -> None:
        measurement_id = str(self.inputs._measurement_id.get("") or "")
        package = self.inputs._measurement_package.get(None)
        run_metadata = self.inputs._measurement_run_metadata.get(None)
        config_owner = package if package is not None else run_metadata
        config_token = (
            measurement_id,
            id(config_owner) if config_owner is not None else 0,
        )
        if read(self.state._measurement_method_config_token, None) != config_token:
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
            settings = self.inputs.settings.get(None)
            setting_reader = self.inputs._workstation_setting.get(None)
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
            self.state._measurement_preferred_method_id = (
                normalize_measurement_method_id(
                    metadata.get("measurement_preferred_method", global_preferred)
                )
            )
            self.state._measurement_active_fallback_order = (
                normalize_measurement_method_fallback_order(
                    metadata.get("measurement_method_fallback_order", global_order)
                )
            )
            self.state._measurement_baseline_yaw_reverse = bool(
                metadata.get("measurement_baseline_yaw_reverse", global_reverse)
            )
            global_time_basis = self.actions._configured_time_basis()
            self.capture_state._measurement_active_time_basis_requested = (
                normalize_live_distance_time_basis(
                    metadata.get(
                        "measurement_pairing_time_basis_requested", global_time_basis
                    )
                )
            )
            self.state._measurement_method_fill_policy = (
                normalize_measurement_method_fill_policy(
                    metadata.get("measurement_method_fill_policy")
                )
            )
            frozen_inputs = metadata.get("measurement_frozen_comparison_inputs")
            self.state._measurement_frozen_comparison_inputs_declared = (
                "measurement_frozen_comparison_inputs" in metadata
            )
            self.state._measurement_frozen_comparison_inputs = (
                dict(frozen_inputs) if isinstance(frozen_inputs, Mapping) else {}
            )
            self.state._measurement_method_config_token = config_token
        if not self.state._measurement_selected_method_id is not UNSET:
            self.state._measurement_selected_method_id = ""
            self.state._measurement_selected_instance_id = ""
            self.state._measurement_selection_reason = "startup"
            self.state._measurement_method_transition_sequence = 0
            self.state._measurement_method_ready_since = {}

    def measurement_method_config_for_new_measurement(self) -> dict[str, object]:
        """Freeze global method defaults into a new Measurement boundary."""
        preferred = normalize_measurement_method_id(
            self.inputs._workstation_setting.get()(
                "measurement_preferred_method",
                getattr(
                    self.inputs.settings.get(),
                    "measurement_preferred_method",
                    PIXHAWK_RTK_LIVE,
                ),
            )
        )
        order = normalize_measurement_method_fallback_order(
            self.inputs._workstation_setting.get()(
                "measurement_method_fallback_order",
                getattr(
                    self.inputs.settings.get(),
                    "measurement_method_fallback_order",
                    DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
                ),
            )
        )
        owner = self.inputs.optical
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
        frozen_intrinsics_result = self.inputs.active_intrinsics.get(None)
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
        self.state._pending_measurement_frozen_intrinsics_result = (
            None
            if frozen_intrinsics_result is None
            else without_stereo_extrinsics(frozen_intrinsics_result)
        )
        self.state._pending_measurement_frozen_intrinsics_path = str(
            getattr(operating_profile, "source_path", "")
            or self.inputs._active_intrinsics_profile_path.get("")
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
        checkerboard_draft = self.inputs.checkerboard_extrinsics_candidate_profile.get(
            None
        )
        if checkerboard_draft is not None:
            frozen_inputs[CHECKERBOARD_STEREO] = {
                "kind": "checkerboard_draft",
                "instance_id": f"frozen-draft:{checkerboard_draft.profile_id or checkerboard_draft.content_sha256}",
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
                if callable(mapper) and isinstance(prior, tuple) and (len(prior) == 2)
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
            json.dumps(frozen_inputs, ensure_ascii=False, allow_nan=False, default=str)
        )
        self.state._pending_measurement_frozen_comparison_objects = frozen_objects
        self.state._measurement_frozen_comparison_inputs = frozen_inputs
        return {
            "measurement_method_config_schema": "measurement-method-config-v1",
            "measurement_preferred_method": preferred,
            "measurement_method_fallback_order": list(order),
            "measurement_method_fill_policy": default_measurement_method_fill_policy(),
            "measurement_baseline_yaw_reverse": bool(
                self.inputs._workstation_setting.get()(
                    "measurement_baseline_yaw_reverse",
                    getattr(
                        self.inputs.settings.get(),
                        "measurement_baseline_yaw_reverse",
                        False,
                    ),
                )
            ),
            "measurement_frozen_comparison_inputs": frozen_inputs,
            "measurement_pairing_time_basis_requested": self.actions._configured_time_basis(),
        }

    def _measurement_fill_policy(self) -> dict[str, dict[str, str]]:
        self._measurement_method_state()
        return normalize_measurement_method_fill_policy(
            read(self.state._measurement_method_fill_policy, None)
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
            self.actions._cached_measurement_attitude_states()
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
        pixhawk_hard = tuple((reason for reason in hard_reasons if "Pixhawk" in reason))
        pixhawk_ready = not pixhawk_hard
        gnss_ready = False
        left_gps, right_gps, reused_gps = self.actions._cached_measurement_gps_states()
        dropped_frame_sides = tuple(
            (
                str(side)
                for side in read(
                    self.capture_state._measurement_gps_correction_frame_dropped, ()
                )
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
        elif (
            not left_gps.has_position
            or not right_gps.has_position
            or (
                not all(
                    (
                        math.isfinite(float(value))
                        for sample in (left_gps, right_gps)
                        for value in (
                            sample.latitude_deg,
                            sample.longitude_deg,
                            sample.altitude_m,
                        )
                    )
                )
            )
        ):
            hard_reasons.append("GNSS position missing or non-finite")
        else:
            ned = ned_delta_m(left_gps, right_gps)
            baseline_m = (
                float("nan")
                if ned is None
                else math.sqrt(sum((float(value) ** 2 for value in ned)))
            )
            if not math.isfinite(baseline_m) or baseline_m <= 0.0:
                hard_reasons.append("GPS baseline length <= 0 or non-finite")
            else:
                gnss_ready = True
                baseline = self.inputs.latest_rtk_baseline.get(None)
                advisories.extend(tuple(getattr(baseline, "warnings", ()) or ()))
                usability = self.actions.panel__gps_baseline_usable_for_translation.get(
                    None
                )
                if baseline is not None and callable(usability):
                    _usable, reason = usability(baseline)
                    if reason and (not _usable):
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
                try:
                    pair_delta_s = abs(
                        float(left_gps.timestamp_s) - float(right_gps.timestamp_s)
                    )
                except (AttributeError, TypeError, ValueError):
                    pair_delta_s = float("nan")
                maximum_pair_delta_s = (
                    float(
                        getattr(
                            self.actions._gnss_readiness_config(),
                            "maximum_left_right_timestamp_delta_ms",
                            defaults.maximum_left_right_timestamp_delta_ms,
                        )
                    )
                    * 0.001
                )
                if math.isfinite(pair_delta_s) and pair_delta_s > maximum_pair_delta_s:
                    advisories.append(
                        f"P1/P2 GPS pair captured {pair_delta_s:.3f}s apart (limit {maximum_pair_delta_s:.3f}s)"
                    )
        if gnss_ready and bool(self.inputs.gnss_operation_gate_active.get(False)):
            manager = self.inputs.gnss_operation_manager.get(None)
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
        candidate: object, side: str | None = None
    ) -> tuple[str, ...]:
        mask = getattr(candidate, "optimized_dofs", None)
        side_rows = dict(getattr(candidate, "observability_by_side", ()) or ())
        observable = dict(
            side_rows.get(side, getattr(candidate, "observability", ()) or ())
        )
        return tuple(
            (
                name
                for name in POSE_DOF_NAMES
                if bool(getattr(mask, name, False))
                and bool(observable.get(name, False))
            )
        )

    def _operating_intrinsics_readiness(
        self, owner: object
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
        if self.inputs.active_intrinsics.get(None) is None and (not nominal_active):
            reasons.append("exact operating K,D missing")
        row = applicability if isinstance(applicability, Mapping) else {}
        geometry_allowed = bool(row.get("geometry_allowed"))
        if not geometry_allowed:
            reasons.append("exact operating K,D not applicable")
            reasons.extend(
                (
                    f"K,D applicability: {reason}"
                    for reason in tuple(row.get("reasons") or ())
                )
            )
        profile_hash = str(row.get("profile_sha256") or "")
        if not (
            len(profile_hash) == 64
            and all((character in "0123456789abcdef" for character in profile_hash))
        ):
            reasons.append("exact operating K,D profile SHA-256 missing or invalid")
            profile_hash = ""
        stable_reasons = tuple(dict.fromkeys(reasons))
        return (not stable_reasons, profile_hash, stable_reasons)

    def _measurement_baseline_yaw_reverse(self) -> bool:
        self._measurement_method_state()
        setting_reader = self.inputs._workstation_setting.get(None)
        if not callable(setting_reader):
            setting_reader = lambda _name, default=None: default
        settings = self.inputs.settings.get(None)
        default = getattr(settings, "measurement_baseline_yaw_reverse", False)
        if not str(self.inputs._measurement_id.get("") or ""):
            return bool(setting_reader("measurement_baseline_yaw_reverse", default))
        if self.state._measurement_baseline_yaw_reverse is not UNSET:
            return bool(self.actions.panel__measurement_baseline_yaw_reverse.get(None))
        return bool(setting_reader("measurement_baseline_yaw_reverse", default))

    def _measurement_method_statuses(self) -> tuple[MeasurementMethodStatus, ...]:
        """Project every geometry source into independent method readiness."""
        self._measurement_method_state()
        owner = self.inputs.optical
        frozen_inputs = read(self.state._measurement_frozen_comparison_inputs, {})
        frozen_inputs = frozen_inputs if isinstance(frozen_inputs, Mapping) else {}
        frozen_objects = read(
            self.state._pending_measurement_frozen_comparison_objects, {}
        )
        frozen_objects = frozen_objects if isinstance(frozen_objects, Mapping) else {}
        measurement_active = bool(
            str(self.inputs._measurement_id.get("") or "")
            and bool(
                read(self.state._measurement_frozen_comparison_inputs_declared, False)
            )
        )
        _geometry_allowed, current_profile_hash, intrinsics_reasons = (
            self._operating_intrinsics_readiness(owner)
        )
        if measurement_active:
            primary_input = frozen_inputs.get(PIXHAWK_RTK_LIVE)
            primary_input = primary_input if isinstance(primary_input, Mapping) else {}
            frozen_kd = primary_input.get("operating_intrinsics")
            frozen_kd = frozen_kd if isinstance(frozen_kd, Mapping) else {}
            current_profile_hash = str(frozen_kd.get("profile_sha256") or "")
            frozen_reasons: list[str] = []
            if not bool(frozen_kd.get("geometry_allowed", False)):
                frozen_reasons.append("frozen exact operating K,D not applicable")
                frozen_reasons.extend(
                    (
                        f"K,D applicability: {reason}"
                        for reason in frozen_kd.get("reasons", ()) or ()
                    )
                )
            if not (
                len(current_profile_hash) == 64
                and all(
                    (
                        character in "0123456789abcdef"
                        for character in current_profile_hash
                    )
                )
            ):
                frozen_reasons.append(
                    "frozen exact operating K,D profile SHA-256 missing or invalid"
                )
                current_profile_hash = ""
            if (
                read(self.state._pending_measurement_frozen_intrinsics_result, None)
                is None
            ):
                frozen_reasons.append("frozen exact operating K,D values missing")
            intrinsics_reasons = tuple(dict.fromkeys(frozen_reasons))
        pixhawk_ready, gnss_ready, live_hard_reasons, live_advisories = (
            self._measurement_live_readiness()
        )
        statuses: list[MeasurementMethodStatus] = []
        pixhawk_reasons = list(intrinsics_reasons)
        if not pixhawk_ready:
            pixhawk_reasons.extend(
                (reason for reason in live_hard_reasons if "Pixhawk" in reason)
            )
        if not gnss_ready:
            pixhawk_reasons.extend(
                (reason for reason in live_hard_reasons if "Pixhawk" not in reason)
            )
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
                        (
                            f"{side}.{name}",
                            "live_pixhawk" if name.endswith("_deg") else "live_gnss",
                        )
                        for side in ("left", "right")
                        for name in POSE_DOF_NAMES
                    )
                ),
                requires_live_pixhawk=True,
                requires_live_gnss=True,
                warnings=live_advisories,
            )
        )
        baseline_yaw_selection, _p1_yaw_age_s, _p2_yaw_age_s = (
            self.actions._baseline_yaw_selection_preview()
        )
        baseline_yaw_reasons = list(intrinsics_reasons)
        if not pixhawk_ready:
            baseline_yaw_reasons.extend(
                (reason for reason in live_hard_reasons if "Pixhawk" in reason)
            )
        if not gnss_ready:
            baseline_yaw_reasons.extend(
                (reason for reason in live_hard_reasons if "Pixhawk" not in reason)
            )
        if baseline_yaw_selection is None or not baseline_yaw_selection.ready:
            baseline_yaw_reasons.append(
                "baseline yaw branch unavailable"
                if baseline_yaw_selection is None
                else baseline_yaw_selection.reason
            )
        baseline_yaw_reverse = self._measurement_baseline_yaw_reverse()
        baseline_yaw_sources = (
            tuple(((f"{side}.roll_deg", "live_pixhawk") for side in ("left", "right")))
            + tuple(
                ((f"{side}.pitch_deg", "live_pixhawk") for side in ("left", "right"))
            )
            + tuple(
                (
                    (
                        f"{side}.yaw_deg",
                        (
                            "gnss_baseline_yaw_reverse"
                            if baseline_yaw_reverse
                            else "gnss_baseline_yaw_auto"
                        ),
                    )
                    for side in ("left", "right")
                )
            )
            + tuple(
                (
                    (f"{side}.{name}", "live_gnss")
                    for side in ("left", "right")
                    for name in ("north_m", "east_m", "down_m")
                )
            )
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
                geometry_sha256=self.actions._geometry_evidence_sha256(
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
        checkerboard_row = (
            checkerboard_row if isinstance(checkerboard_row, Mapping) else {}
        )
        checkerboard_profile = (
            frozen_objects.get(CHECKERBOARD_STEREO)
            if measurement_active
            else self.inputs.checkerboard_extrinsics_profile.get(None)
        )
        checkerboard_reasons = list(intrinsics_reasons)
        if checkerboard_profile is None:
            if measurement_active:
                checkerboard_reasons.append("frozen checkerboard Draft missing")
            elif (
                self.inputs.checkerboard_extrinsics_candidate_profile.get(None)
                is not None
            ):
                checkerboard_reasons.append(
                    "checkerboard stereo R,t candidate not applied"
                )
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
                profile_id=(
                    ""
                    if checkerboard_profile is None
                    else checkerboard_profile.profile_id
                ),
                geometry_sha256=(
                    ""
                    if checkerboard_profile is None or not current_profile_hash
                    else self.actions._geometry_evidence_sha256(
                        operating_intrinsics_profile_sha256=current_profile_hash,
                        stereo_extrinsics_sha256=checkerboard_profile.content_sha256,
                    )
                ),
                observed_dofs=POSE_DOF_NAMES,
                component_sources=tuple(
                    (
                        (f"{side}.{name}", "checkerboard")
                        for side in ("left", "right")
                        for name in POSE_DOF_NAMES
                    )
                ),
            )
        )
        active_selection = getattr(owner, "_active_selection", None)
        active_candidate_id = str(getattr(active_selection, "candidate_id", "") or "")
        active_selected_dofs = set(
            getattr(
                getattr(active_selection, "selected_dofs", None), "selected_names", ()
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
                        (
                            name
                            for name in observed_by_side[side]
                            if name in active_selected_dofs
                        )
                    )
                    for side in ("left", "right")
                }
            observed = tuple(
                (
                    name
                    for name in POSE_DOF_NAMES
                    if all(
                        (name in observed_by_side[side] for side in ("left", "right"))
                    )
                )
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
            elif method_id == CLOUD_GNSS:
                for side in ("left", "right"):
                    if not {"roll_deg", "pitch_deg", "yaw_deg"} <= set(
                        observed_by_side[side]
                    ):
                        reasons.append(
                            f"{side} Cloud far-field rotation not observable"
                        )
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
            fillable_method = method_id in {
                HORIZON_HYBRID,
                DRONE_RTK_ROTATION_HYBRID,
                CLOUD_GNSS,
            }
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
            if requires_live_pixhawk and (not pixhawk_ready):
                reasons.append("fresh live Pixhawk required for missing rotation axes")
                reasons.extend(
                    (reason for reason in live_hard_reasons if "Pixhawk" in reason)
                )
            if requires_live_gnss and (not gnss_ready):
                reasons.append("usable live GNSS required for missing translation axes")
                reasons.extend(
                    (reason for reason in live_hard_reasons if "Pixhawk" not in reason)
                )
            component_sources = tuple(
                (
                    (f"{side}.{name}", fill_sources[f"{side}.{name}"])
                    for side in ("left", "right")
                    for name in POSE_DOF_NAMES
                )
            )
            statuses.append(
                MeasurementMethodStatus(
                    method_id=method_id,
                    instance_id=str(frozen_row.get("instance_id") or "")
                    or f"candidate:{candidate.candidate_id}",
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
                    warnings=tuple(
                        dict.fromkeys((*candidate_warnings, *live_advisories))
                    ),
                    requires_live_pixhawk=requires_live_pixhawk,
                    requires_live_gnss=requires_live_gnss,
                )
            )
        loaded_profile = self.inputs.optical_pose_extrinsics_profile.get(None)
        loaded_method_id = (
            None
            if loaded_profile is None
            else measurement_method_id_for_candidate(loaded_profile.method)
        )
        if (
            not measurement_active
            and loaded_profile is not None
            and (loaded_method_id is not None)
        ):
            loaded_instance = (
                f"profile:{loaded_profile.profile_id or loaded_profile.content_sha256}"
            )
            if not any((row.instance_id == loaded_instance for row in statuses)):
                reasons = list(intrinsics_reasons)
                loaded_metadata = getattr(loaded_profile, "metadata", {})
                loaded_metadata = (
                    loaded_metadata if isinstance(loaded_metadata, Mapping) else {}
                )
                expected_profile_hash = str(
                    loaded_metadata.get("operating_intrinsics_profile_sha256", "") or ""
                )
                loaded_warnings: list[str] = []
                if not expected_profile_hash:
                    reasons.append(
                        "applied optical profile operating K,D SHA-256 missing"
                    )
                elif expected_profile_hash != current_profile_hash:
                    reasons.append(
                        "applied optical profile K,D differs from active K,D"
                    )
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
                            else self.actions._geometry_evidence_sha256(
                                operating_intrinsics_profile_sha256=current_profile_hash,
                                stereo_extrinsics_sha256=loaded_profile.content_sha256,
                            )
                        ),
                        observed_dofs=POSE_DOF_NAMES,
                        component_sources=tuple(
                            (
                                (
                                    f"{side}.{name}",
                                    f"profile:{loaded_profile.profile_id}",
                                )
                                for side in ("left", "right")
                                for name in POSE_DOF_NAMES
                            )
                        ),
                        warnings=tuple(
                            dict.fromkeys((*loaded_warnings, *live_advisories))
                        ),
                    )
                )
        return tuple(statuses)

    def _measurement_fallback_order(self) -> tuple[str, ...]:
        self._measurement_method_state()
        return normalize_measurement_method_fallback_order(
            read(
                self.state._measurement_active_fallback_order,
                DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
            )
        )

    def _measurement_fill_policy_locked(self) -> bool:
        state = str(
            self.inputs._measurement_package_state.get("IDLE") or "IDLE"
        ).upper()
        measurement_id = str(self.inputs._measurement_id.get("") or "")
        detection_started = bool(
            self.inputs._measurement_package_detection_started.get(False)
        )
        return not (
            measurement_id and state == "COLLECTING" and (not detection_started)
        )

    def _measurement_component_controls(
        self, status: MeasurementMethodStatus
    ) -> dict[str, dict[str, object]]:
        sources = dict(status.component_sources)
        pixhawk_ready, gnss_ready, _hard_reasons, _advisories = (
            self._measurement_live_readiness()
        )
        locked = self._measurement_fill_policy_locked()
        rows: dict[str, dict[str, object]] = {}
        atomic_method = status.method_id in {
            PIXHAWK_RTK_LIVE,
            PIXHAWK_RP_GPS_BASELINE_YAW,
            CHECKERBOARD_STEREO,
        } or bool(status.profile_id and (not status.candidate_id))
        for side in MEASUREMENT_POSE_SIDES:
            for dof in MEASUREMENT_POSE_DOFS:
                key = f"{side}.{dof}"
                source = str(sources.get(key) or MEASUREMENT_FILL_NONE)
                candidate_provided = source.startswith("candidate:")
                provided = bool(
                    atomic_method or candidate_provided or source.startswith("profile:")
                )
                essential_missing = bool(
                    status.candidate_id
                    and (
                        status.method_id == HORIZON_HYBRID
                        and dof in {"roll_deg", "pitch_deg"}
                        or (
                            status.method_id == DRONE_RTK_ROTATION_HYBRID
                            and dof == "yaw_deg"
                        )
                        or (
                            status.method_id == CLOUD_GNSS
                            and dof in {"roll_deg", "pitch_deg", "yaw_deg"}
                        )
                        or (
                            status.method_id
                            in {
                                MOUNTAIN_GNSS,
                                DRONE_RTK_MULTIVIEW,
                                TERRAIN_PNP,
                                COMBINED_HYBRID,
                            }
                        )
                    )
                    and (not candidate_provided)
                )
                compatible = compatible_measurement_fill_source(dof)
                editable = bool(not provided and (not essential_missing))
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
                        else (
                            "method-defining DOF is not observable"
                            if essential_missing
                            else (
                                "fill source selected"
                                if source == compatible
                                else "fill disabled"
                            )
                        )
                    ),
                }
        return rows

    def configure_workstation_measurement_method_fill(
        self, payload: object
    ) -> dict[str, object]:
        self._measurement_method_state()
        if self._measurement_fill_policy_locked():
            raise RuntimeError(
                "DOF fill policy is editable only before Detection starts"
            )
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
            raise ValueError(
                f"measurement component is method-owned: {method_id}/{component}"
            )
        enabled = bool(values.get("enabled", True))
        source = (
            compatible_measurement_fill_source(dof)
            if enabled
            else MEASUREMENT_FILL_NONE
        )
        policy = self._measurement_fill_policy()
        policy[method_id][component] = source
        self.state._measurement_method_fill_policy = (
            normalize_measurement_method_fill_policy(policy)
        )
        package = self.inputs._measurement_package.get(None)
        updater = getattr(package, "update_metadata", None)
        if callable(updater):
            updater(
                {
                    "measurement_method_fill_policy": read(
                        self.state._measurement_method_fill_policy
                    )
                }
            )
        writer = self.inputs.append_workstation_run_event.get(None)
        if callable(writer):
            writer(
                "measurement_method_fill_changed",
                {
                    "measurement_id": str(self.inputs._measurement_id.get("") or ""),
                    "method_id": method_id,
                    "component": component,
                    "source": source,
                },
                source="MeasurementViewController",
                coordinate_contract="per-Measurement virtual-FRD RPY / NED center fill policy",
            )
        self.state._measurement_selected_method_id = ""
        self.state._measurement_selected_instance_id = ""
        self.state._measurement_selection_reason = "fill_policy_changed"
        self.state._measurement_method_ready_since = {}
        return self.workstation_measurement_methods_snapshot()

    def _set_selected_measurement_method(
        self, status: MeasurementMethodStatus, *, reason: str
    ) -> None:
        self._measurement_method_state()
        previous_method = read(self.state._measurement_selected_method_id)
        previous_instance = read(self.state._measurement_selected_instance_id)
        if (
            previous_method == status.method_id
            and previous_instance == status.instance_id
        ):
            return
        self.state._measurement_selected_method_id = status.method_id
        self.state._measurement_selected_instance_id = status.instance_id
        self.state._measurement_selection_reason = str(reason)
        self.state._measurement_method_transition_sequence += 1
        self.geometry_state._last_measurement_attempt_key = None
        self.geometry_state._last_measurement_attempt = None
        writer = self.inputs.append_workstation_run_event.get(None)
        if callable(writer):
            writer(
                "measurement_method_switched",
                {
                    "sequence": read(
                        self.state._measurement_method_transition_sequence
                    ),
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
        previous_method = read(self.state._measurement_selected_method_id)
        previous_instance = read(self.state._measurement_selected_instance_id)
        if not previous_method and (not previous_instance):
            self.state._measurement_selection_reason = str(reason)
            return
        self.state._measurement_selected_method_id = ""
        self.state._measurement_selected_instance_id = ""
        self.state._measurement_selection_reason = str(reason)
        self.state._measurement_method_transition_sequence += 1
        self.geometry_state._last_measurement_attempt_key = None
        self.geometry_state._last_measurement_attempt = None
        writer = self.inputs.append_workstation_run_event.get(None)
        if callable(writer):
            writer(
                "measurement_method_unavailable",
                {
                    "sequence": read(
                        self.state._measurement_method_transition_sequence
                    ),
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
        statuses = self._measurement_method_statuses()
        now_s = time.monotonic()
        ready_keys = {(row.method_id, row.instance_id) for row in statuses if row.ready}
        ready_since = read(self.state._measurement_method_ready_since)
        for key in tuple(ready_since):
            if key not in ready_keys:
                ready_since.pop(key, None)
        for key in ready_keys:
            ready_since.setdefault(key, now_s)
        current = next(
            (
                row
                for row in statuses
                if row.method_id == read(self.state._measurement_selected_method_id)
                and row.instance_id
                == read(self.state._measurement_selected_instance_id)
            ),
            None,
        )
        preferred = read(self.state._measurement_preferred_method_id)
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
        if (current.method_id, current.instance_id) == (
            chosen.method_id,
            chosen.instance_id,
        ):
            return current
        key = (chosen.method_id, chosen.instance_id)
        since = float(ready_since[key])
        if now_s - since < 1.0:
            return current
        reason = (
            "auto_recovery" if chosen.method_id == preferred else "priority_preemption"
        )
        self._set_selected_measurement_method(chosen, reason=reason)
        return chosen

    def _measurement_method_input_snapshot(
        self, status: MeasurementMethodStatus
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
        live_attitudes = self.inputs.latest_extrinsics.get({}) or {}
        for side in MEASUREMENT_POSE_SIDES:
            sample = (
                live_attitudes.get(side)
                if isinstance(live_attitudes, Mapping)
                else None
            )
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
        baseline = self.inputs.latest_rtk_baseline.get(None)
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
            profile = self.inputs.checkerboard_extrinsics_profile.get(None)
        elif status.profile_id and (not status.candidate_id):
            loaded = self.inputs.optical_pose_extrinsics_profile.get(None)
            if (
                loaded is not None
                and str(getattr(loaded, "profile_id", "")) == status.profile_id
            ):
                profile = loaded
        extrinsics_row: object = None
        extrinsics = None if profile is None else getattr(profile, "extrinsics", None)
        if extrinsics is not None:
            extrinsics_row = {
                "rotation_left_to_right": getattr(
                    extrinsics, "rotation_left_to_right", None
                ),
                "translation_left_to_right_m": getattr(
                    extrinsics, "translation_left_to_right_m", None
                ),
                "frame_contract": str(getattr(extrinsics, "frame_contract", "") or ""),
                "profile_id": str(getattr(profile, "profile_id", "") or ""),
            }
        baseline_yaw_row: object = None
        if status.method_id == PIXHAWK_RP_GPS_BASELINE_YAW:
            selection, p1_age_s, p2_age_s = (
                self.actions._baseline_yaw_selection_preview()
            )
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
            read(self.state._measurement_frozen_comparison_inputs_declared, False)
        )
        selected = (
            latest_status_by_method(self._measurement_method_statuses()).get(
                normalize_measurement_method_id(
                    read(self.state._measurement_selected_method_id, "")
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
        rank = {method_id: index + 1 for (index, method_id) in enumerate(order)}
        ready.sort(key=lambda row: (rank.get(row.method_id, len(rank) + 1), row.label))
        capability_rows = {
            method_id: {
                **row.to_mapping(),
                "components": self._measurement_component_controls(row),
                "input_values": self._measurement_method_input_snapshot(row),
                "required_inputs": list(MEASUREMENT_METHOD_REQUIRED_INPUTS[method_id]),
            }
            for (method_id, row) in latest.items()
        }
        method_catalog = []
        last_bundle = read(self.commit_state._last_method_evaluation_bundle, None)
        last_outcomes = (
            {row.method_id: row for row in last_bundle.outcomes}
            if isinstance(last_bundle, MethodEvaluationBundle)
            else {}
        )
        counters = read(self.commit_state._measurement_method_result_counters, {})
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
                    "required_inputs": list(
                        MEASUREMENT_METHOD_REQUIRED_INPUTS[method_id]
                    ),
                }
            outcome = last_outcomes.get(method_id)
            if outcome is not None:
                runtime_status = (
                    "REUSED"
                    if outcome.reused_inputs
                    and outcome.status in {"VALID", "ESTIMATED"}
                    else (
                        "ACTIVE"
                        if outcome.status in {"VALID", "ESTIMATED"}
                        else (
                            "BLOCKED"
                            if outcome.status in {"BLOCKED", "BACKPRESSURE_BLOCKED"}
                            else outcome.status
                        )
                    )
                )
                runtime_reasons = list(outcome.reasons)
            else:
                runtime_status = (
                    "ACTIVE"
                    if frozen_measurement and bool(row.get("ready"))
                    else "READY" if bool(row.get("ready")) else "BLOCKED"
                )
                runtime_reasons = list(row.get("reasons") or ())
            row["runtime_status"] = runtime_status
            row["primary"] = method_id == PIXHAWK_RTK_LIVE
            row["frozen_for_measurement"] = frozen_measurement
            row["primary_blocker"] = runtime_reasons[0] if runtime_reasons else ""
            row["additional_blocker_count"] = max(0, len(runtime_reasons) - 1)
            row["runtime_reasons"] = list(runtime_reasons)
            row.setdefault(
                "required_inputs", list(MEASUREMENT_METHOD_REQUIRED_INPUTS[method_id])
            )
            row["result_counters"] = dict(
                counters.get(method_id, {"VALID": 0, "BLOCKED": 0, "FAILED": 0})
            )
            method_catalog.append(row)
        return {
            "schema": "measurement-method-snapshot-v1",
            "preferred_method_id": read(self.state._measurement_preferred_method_id),
            "selected_method_id": "" if selected is None else selected.method_id,
            "selected_instance_id": "" if selected is None else selected.instance_id,
            "selection_reason": read(self.state._measurement_selection_reason),
            "transition_sequence": read(
                self.state._measurement_method_transition_sequence
            ),
            "fallback_order": list(order),
            "measurement_id": str(self.inputs._measurement_id.get("") or ""),
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

    def select_workstation_measurement_method(
        self, payload: object
    ) -> dict[str, object]:
        self._measurement_method_state()
        values = payload if isinstance(payload, dict) else {"method_id": payload}
        method_id = normalize_measurement_method_id(values.get("method_id"))
        ready = {
            row.method_id: row
            for row in latest_status_by_method(
                self._measurement_method_statuses()
            ).values()
            if row.ready
        }
        status = ready.get(method_id)
        if status is None:
            raise ValueError(f"measurement method is not ready: {method_id}")
        self.state._measurement_preferred_method_id = method_id
        package = self.inputs._measurement_package.get(None)
        updater = getattr(package, "update_metadata", None)
        if callable(updater):
            updater({"measurement_preferred_method": method_id})
        self._set_selected_measurement_method(status, reason="manual")
        return self.workstation_measurement_methods_snapshot()

    def _measurement_candidate(self, candidate_id: str):
        owner = self.inputs.optical
        return getattr(owner, "_candidates", {}).get(str(candidate_id))
