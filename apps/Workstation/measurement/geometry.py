"""Geometry resolution from captured inputs; NED/FRD/RDF contracts are explicit.

Attitudes: body FRD -> earth NED, wxyz. GPS baselines: NED metres.
Optical R,t: X_right_RDF = R_right_from_left X_left_RDF + t_m.
Public point_m: baseline-center FRD; source_point_left_rdf_m: left optical RDF.
"""

from __future__ import annotations
import hashlib
import json
import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from math import isfinite
from Workstation.workstation_core.attitude_geometry import (
    relative_attitude,
    world_ned_vector_to_body_frd,
)
from Workstation.workstation_core.canonical_transform import (
    flatten_rotation,
    flatten_translation,
)
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
    CLOUD_GNSS,
    COMBINED_HYBRID,
    DRONE_RTK_MULTIVIEW,
    DRONE_RTK_ROTATION_HYBRID,
    HORIZON_HYBRID,
    MOUNTAIN_GNSS,
    PIXHAWK_RTK_LIVE,
    PIXHAWK_RP_GPS_BASELINE_YAW,
    TERRAIN_PNP,
    MeasurementMethodStatus,
    measurement_method_id_for_candidate,
)
from Workstation.workstation_core.optical_pose import (
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
    GnssReadinessSnapshot,
    ReadinessConfig,
)
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    GeometryConfigSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
    build_checkerboard_baseline_snapshot,
    build_pixhawk_baseline_snapshot,
    detection_pair_key,
)
from Workstation.workstation_core.baseline_yaw import (
    BASELINE_YAW_MINUS_90_DEG,
    BaselineYawSelection,
    quaternion_from_roll_pitch_yaw_deg,
    select_baseline_yaw,
)
from Workstation.workstation_core.method_evaluation import FrozenMeasurementPairSnapshot
from Workstation.workstation_core.rtk_geometry import (
    GpsPairReadiness,
    GpsPositionState,
    RtkBaselineEstimate,
    ned_delta_m,
    validate_gps_pair_for_baseline,
)
from Workstation.workstation_core.triangulation import (
    body_frd_vector_to_camera,
    camera_translation_right_in_left_camera,
    invert_left_to_right_camera_transform,
    triangulate_pixels_with_pixhawk_mount_attempt,
    triangulate_pixels_with_relative_pose_attempt,
)
from typing import Any, Callable
from Workstation.measurement import limits
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.optical_inputs import OpticalMeasurementInputs
from Workstation.measurement.frozen_inputs import (
    FrozenGeometryInputs,
    FrozenMethodGeometry,
)
from Workstation.measurement.geometry_resolver import (
    resolve_measurement_attempt,
    measurement_geometry_hash,
)
from Workstation.measurement.state import (
    UNSET,
    read,
    PairCaptureState,
    GeometryState,
    MethodSelectionState,
)


@dataclass(frozen=True)
class GeometryInputs:
    checkerboard_extrinsics_profile: ValueInput
    optical: OpticalMeasurementInputs
    _active_optical_pose_profile_hash: ValueInput
    _selected_baseline_gps_states: ValueInput
    latest_gps: ValueInput
    active_intrinsics: ValueInput
    optical_pose_extrinsics_profile: ValueInput
    _active_intrinsics_source_zoom_level: ValueInput
    _active_intrinsics_profile_path: ValueInput
    _distance_mode: ValueInput
    latest_rtk_baseline: ValueInput
    gnss_operation_manager: ValueInput
    settings: ValueInput
    _pixhawk_gnss_source: ValueInput
    latest_extrinsics: ValueInput
    baseline_scale_m: Callable[..., Any]
    last_calibration_result: ValueInput
    optical_expected_dy_combo_currentData: Callable[..., Any]
    optical_expected_dy_spin_value: Callable[..., Any]
    active_zoom: Callable[..., Any]
    _active_extrinsics_source_zoom_level: ValueInput
    _yolo_pair_max_delta_s: ValueInput
    _yolo_vertical_max_delta_px: ValueInput
    focal_length_px: Callable[..., Any]
    manual_focal_override: Callable[..., Any]
    _workstation_setting: ValueInput
    gnss_fixed_base_profile: ValueInput
    latest_baseline_vector: ValueInput
    _gnss_base_rtcm_target_sides: ValueInput
    _selected_rover_gps_state: ValueInput
    _fmt_record_float: ValueInput
    workstation_optical_pose_candidate_poses: ValueInput


@dataclass(frozen=True)
class GeometryActions:
    _cached_measurement_attitude_states: Callable[..., Any]
    _measurement_baseline_yaw_reverse: Callable[..., Any]
    _measurement_candidate: Callable[..., Any]
    _candidate_observed_dofs: Callable[..., Any]
    panel__checkerboard_translation_right_in_left_m: ValueInput
    panel__gps_baseline_usable_for_translation: ValueInput
    panel__optical_alignment_intrinsics_result: ValueInput
    _select_measurement_method_for_attempt: Callable[..., Any]
    _measurement_method_statuses: Callable[..., Any]
    _freeze_measurement_pair_inputs: Callable[..., Any]


class GeometryInputAdapter:
    """Geometry resolution from captured inputs; NED/FRD/RDF contracts are explicit."""

    def __init__(
        self,
        *,
        state: GeometryState,
        inputs: GeometryInputs,
        actions: GeometryActions,
        capture_state: PairCaptureState,
        methods_state: MethodSelectionState,
    ) -> None:
        self.state = state
        self.inputs = inputs
        self.actions = actions
        self.capture_state = capture_state
        self.methods_state = methods_state

    def measurement_alignment_rows(self) -> list[dict[str, object]]:
        """R,t actually applied when the Measurement starts (04_Alignment)."""
        rows: list[dict[str, object]] = []
        now_ns = time.time_ns()
        profile = self.inputs.checkerboard_extrinsics_profile.get(None)
        extrinsics = getattr(profile, "extrinsics", None)
        if extrinsics is not None:
            try:
                rows.append(
                    {
                        "timestamp_utc_ns": now_ns,
                        "event_type": "applied_at_start",
                        "method_id": CHECKERBOARD_STEREO,
                        "source": f"profile:{getattr(profile, 'profile_id', '') or ''}",
                        **flatten_rotation(extrinsics.rotation_left_to_right),
                        **flatten_translation(extrinsics.translation_left_to_right_m),
                        "source_sha256": str(
                            getattr(profile, "content_sha256", "") or ""
                        ),
                        "detail": f"reprojection_error_px={getattr(extrinsics, 'reprojection_error_px', '')}; pair_count={getattr(extrinsics, 'pair_count', '')}; method={getattr(profile, 'method', '')}",
                    }
                )
            except (AttributeError, TypeError, ValueError):
                pass
        try:
            owner = self.inputs.optical
            snapshot = owner.workstation_optical_pose_snapshot(include_frames=False)
        except Exception:
            snapshot = None
        try:
            accepted = getattr(owner, "_last_accepted_auto_suggestion", None)
        except Exception:
            accepted = None
        if isinstance(accepted, Mapping):
            from Workstation.optical_pose.artifacts import (
                accepted_suggestion_alignment_row,
            )

            accepted_row = accepted_suggestion_alignment_row(accepted)
            accepted_row["event_type"] = "optical_pose_auto_suggestion_at_start"
            accepted_row["recorded_utc_ns"] = now_ns
            rows.append(accepted_row)
        if isinstance(snapshot, Mapping) and snapshot.get("active_selection"):
            active = snapshot.get("active_selection")
            method = None
            if isinstance(active, Mapping):
                method = measurement_method_id_for_candidate(active.get("method"))
            rows.append(
                {
                    "timestamp_utc_ns": now_ns,
                    "event_type": "optical_pose_active_at_start",
                    "method_id": method or "",
                    "source": str(
                        active.get("candidate_id", "")
                        if isinstance(active, Mapping)
                        else ""
                    ),
                    "source_sha256": str(
                        self.inputs._active_optical_pose_profile_hash.get("") or ""
                    ),
                    "detail": json.dumps(
                        snapshot,
                        ensure_ascii=False,
                        sort_keys=True,
                        default=str,
                        separators=(",", ":"),
                    )[:32000],
                }
            )
        return rows

    @staticmethod
    def _geometry_evidence_sha256(**parts: object) -> str:
        return hashlib.sha256(
            json.dumps(parts, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()

    def _baseline_yaw_selection_preview(
        self,
        *,
        left_gps: object | None = None,
        right_gps: object | None = None,
        left_attitude: object | None = None,
        right_attitude: object | None = None,
    ) -> tuple[BaselineYawSelection | None, float | None, float | None]:
        if left_gps is None or right_gps is None:
            selector = self.inputs._selected_baseline_gps_states.get(None)
            if callable(selector):
                selected_left, selected_right = selector()
            else:
                latest_gps = self.inputs.latest_gps.get({}) or {}
                selected_left = (
                    latest_gps.get("left") if isinstance(latest_gps, Mapping) else None
                )
                selected_right = (
                    latest_gps.get("right") if isinstance(latest_gps, Mapping) else None
                )
            left_gps = left_gps or selected_left
            right_gps = right_gps or selected_right
        if left_gps is None or right_gps is None:
            return (None, None, None)
        baseline_ned = ned_delta_m(left_gps, right_gps)
        if baseline_ned is None:
            return (None, None, None)
        if left_attitude is None or right_attitude is None:
            left_attitude, right_attitude, _reused = (
                self.actions._cached_measurement_attitude_states()
            )
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
                reverse=self.actions._measurement_baseline_yaw_reverse(),
            )
        except (TypeError, ValueError):
            return (None, ages[0], ages[1])
        return (selection, ages[0], ages[1])

    def _measurement_intrinsics_result_for_method(
        self,
        status: MeasurementMethodStatus,
        *,
        left_attitude: AttitudeSnapshot | None = None,
        right_attitude: AttitudeSnapshot | None = None,
        left_gps: GpsSnapshot | None = None,
        right_gps: GpsSnapshot | None = None,
        pose_evidence_out: dict[str, object] | None = None,
    ) -> StereoIntrinsicsResult | None:
        owner = self.inputs.optical
        if bool(
            read(
                self.methods_state._measurement_frozen_comparison_inputs_declared, False
            )
        ):
            intrinsics_only = read(
                self.methods_state._pending_measurement_frozen_intrinsics_result, None
            )
            if intrinsics_only is None:
                return None
        else:
            current = self.inputs.active_intrinsics.get(None)
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
            frozen_objects = read(
                self.methods_state._pending_measurement_frozen_comparison_objects, {}
            )
            profile = (
                frozen_objects.get(CHECKERBOARD_STEREO)
                if bool(
                    read(
                        self.methods_state._measurement_frozen_comparison_inputs_declared,
                        False,
                    )
                )
                and isinstance(frozen_objects, Mapping)
                else self.inputs.checkerboard_extrinsics_profile.get(None)
            )
            return (
                intrinsics_only
                if profile is None
                else compose_stereo_calibration(intrinsics_only, profile)
            )
        if status.profile_id and (not status.candidate_id):
            profile = self.inputs.optical_pose_extrinsics_profile.get(None)
            if profile is not None and profile.profile_id == status.profile_id:
                return compose_stereo_calibration(intrinsics_only, profile)
        candidate = self.actions._measurement_candidate(status.candidate_id)
        if candidate is None:
            return None
        owner = self.inputs.optical
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
                    (
                        left_anchor[index] + float(baseline_ned[index])
                        for index in range(3)
                    )
                ),
            }
            attitude_by_side = {"left": left_attitude, "right": right_attitude}
            drone_context = getattr(owner, "_candidate_drone_rotation_context", {}).get(
                candidate.candidate_id, {}
            )
            poses = []
            for side, candidate_pose in zip(
                ("left", "right"), (candidate.left_pose_ned, candidate.right_pose_ned)
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
                elif status.method_id == CLOUD_GNSS:
                    observed = set(
                        self.actions._candidate_observed_dofs(candidate, side)
                    )
                    if not {"roll_deg", "pitch_deg", "yaw_deg"} <= observed:
                        return None
                    final_values = PoseParametersNed(
                        candidate_values.roll_deg,
                        candidate_values.pitch_deg,
                        candidate_values.yaw_deg,
                        *live_centers[side],
                    )
                elif status.method_id == DRONE_RTK_ROTATION_HYBRID:
                    observed = set(
                        self.actions._candidate_observed_dofs(candidate, side)
                    )
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
        if pose_evidence_out is not None:
            from Workstation.optical_pose.geometry_values import _pose_mapping

            pose_evidence_out.update(
                {
                    "schema": "measurement-optical-pose-evidence-v1",
                    "candidate_id": candidate.candidate_id,
                    "yaw90_reference": (
                        json.loads(candidate.reference_evidence_json)
                        if candidate.reference_evidence_json
                        else None
                    ),
                    "target_origin_model": "P1_GNSS_as_left_optical_center",
                    "left_role": "assumed_reference",
                    "right_role": "derived_from_reference_and_images",
                    "left_pose": _pose_mapping(poses[0]),
                    "right_pose": _pose_mapping(poses[1]),
                    "translation_source": (
                        "pair_gnss"
                        if status.method_id == CLOUD_GNSS
                        else "capture_gnss"
                    ),
                    "R_right_from_left_rdf": [
                        list(row) for row in stereo.rotation_right_from_left_rdf
                    ],
                    "t_right_from_left_rdf_m": list(
                        stereo.translation_right_from_left_rdf_m
                    ),
                }
            )
        candidate_quality = dict(candidate.quality_metrics)
        stereo_extrinsics = StereoExtrinsics(
            rotation_left_to_right=[
                list(row) for row in stereo.rotation_right_from_left_rdf
            ],
            translation_left_to_right_m=list(stereo.translation_right_from_left_rdf_m),
            reprojection_error_px=float(
                candidate_quality.get("reprojection_error_px", 0.0)
            ),
            pair_count=len(candidate.source_observation_ids),
        )
        profile = StereoExtrinsicsProfile(
            extrinsics=stereo_extrinsics,
            source_zoom_level=self.inputs._active_intrinsics_source_zoom_level.get(
                None
            ),
            method=f"measurement/{candidate.method}",
            source_intrinsics_path=str(
                self.inputs._active_intrinsics_profile_path.get("") or ""
            ),
            source_intrinsics_sha256=intrinsics_content_sha256(intrinsics_only),
            profile_id=status.instance_id,
            metadata={
                "measurement_method_id": status.method_id,
                "candidate_id": candidate.candidate_id,
                "component_sources": dict(status.component_sources),
            },
        )
        return compose_stereo_calibration(intrinsics_only, profile)

    _measurement_geometry_hash = staticmethod(measurement_geometry_hash)

    def _checkerboard_translation_right_in_left_m(
        self,
    ) -> tuple[float, float, float] | None:
        calibration = self.inputs.active_intrinsics.get()
        extrinsics = None if calibration is None else calibration.extrinsics
        if extrinsics is None:
            return None
        _rotation_right_to_left, translation_right_in_left_m = (
            invert_left_to_right_camera_transform(
                extrinsics.rotation_left_to_right,
                extrinsics.translation_left_to_right_m,
            )
        )
        return translation_right_in_left_m

    def _active_baseline_m(self) -> tuple[float, str, tuple[str, ...]]:
        if self.inputs._distance_mode.get()() == "checkerboard":
            translation = (
                self.actions.panel__checkerboard_translation_right_in_left_m.get(None)()
            )
            if translation is None:
                return (
                    0.0,
                    "checkerboard-waiting",
                    ("waiting for checkerboard stereo extrinsics",),
                )
            baseline_m = math.sqrt(sum((value * value for value in translation)))
            if baseline_m <= 0.0:
                return (
                    0.0,
                    "checkerboard-waiting",
                    ("checkerboard baseline length <= 0",),
                )
            return (baseline_m, "checkerboard", ())
        baseline = self.inputs.latest_rtk_baseline.get()
        if baseline is None:
            return (0.0, "gps-rtk-waiting", ("waiting for live GPS baseline",))
        usable, reason = self.actions.panel__gps_baseline_usable_for_translation.get(
            None
        )(baseline)
        if not usable:
            return (
                baseline.baseline_m,
                "gps-rtk-blocked",
                (*baseline.warnings, reason),
            )
        return (baseline.baseline_m, "gps-rtk", baseline.warnings)

    def _gps_fix_ready_for_baseline(self, state: GpsPositionState | None) -> bool:
        defaults = ReadinessConfig()
        manager = self.inputs.gnss_operation_manager.get(None)
        config = getattr(manager, "config", None)
        if config is None:
            config = getattr(self.inputs.settings.get(None), "gnss_operation", None)
        readiness = getattr(config, "readiness", None)
        minimum_fix_type = int(
            getattr(readiness, "minimum_fix_type", defaults.minimum_fix_type)
        )
        return (
            state is not None
            and state.has_position
            and (state.fix_type is not None)
            and (state.fix_type >= minimum_fix_type)
        )

    def _gps_pair_readiness(
        self, left: GpsPositionState | None, right: GpsPositionState | None
    ) -> GpsPairReadiness:
        defaults = ReadinessConfig()
        manager = self.inputs.gnss_operation_manager.get(None)
        config = getattr(manager, "config", None)
        if config is None:
            config = getattr(self.inputs.settings.get(None), "gnss_operation", None)
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

    def _gps_baseline_usable_for_translation(
        self, baseline: RtkBaselineEstimate | None
    ) -> tuple[bool, str]:
        if baseline is None:
            return (False, "live GPS baseline waiting")
        selector = self.inputs._selected_baseline_gps_states.get(None)
        if not callable(selector):
            return (False, "live GPS pair source unavailable")
        left, right = selector()
        readiness = self._gps_pair_readiness(left, right)
        if not readiness.usable:
            return (False, "; ".join(readiness.reasons))
        if baseline.baseline_m <= 0.0:
            return (False, "GPS baseline length <= 0")
        return (True, "usable")

    def _rtk_fixed_status_text(self, baseline: RtkBaselineEstimate | None) -> str:
        selector = self.inputs._selected_baseline_gps_states.get(None)
        if callable(selector):
            left, right = selector()
        else:
            left = self.inputs.latest_gps.get().get("left")
            right = self.inputs.latest_gps.get().get("right")
        source_reader = self.inputs._pixhawk_gnss_source.get(None)
        left_source = str(source_reader("left")) if callable(source_reader) else "GPS1"
        right_source = (
            str(source_reader("right")) if callable(source_reader) else "GPS1"
        )

        def side_text(label: str, state: GpsPositionState | None) -> str:
            if state is None:
                return f"{label}=대기"
            sats = "--" if state.satellites_used is None else str(state.satellites_used)
            eph = "--" if state.eph_m is None else f"{state.eph_m:.2f}m"
            return f"{label}={state.rtk_label}(fix={state.fix_type}, sats={sats}, eph={eph})"

        fixed = (
            left is not None
            and right is not None
            and (left.fix_type == 6)
            and (right.fix_type == 6)
        )
        usable, reason = self.actions.panel__gps_baseline_usable_for_translation.get(
            None
        )(baseline)
        fixed_text = "YES" if fixed else "NO"
        usability = "사용 가능" if usable else f"대기/차단: {reason}"
        return f"RTK Fixed: {fixed_text} | GPS baseline 필수, {usability} | {side_text(f'P1 {left_source}', left)} | {side_text(f'P2 {right_source}', right)}"

    def _baseline_forward_geometry_from_camera_translation(
        self, camera_translation: tuple[float, float, float] | None
    ) -> tuple[float, float, float, float] | None:
        if camera_translation is None:
            return None
        right_m, down_m, forward_m = camera_translation
        length_m = math.sqrt(
            right_m * right_m + down_m * down_m + forward_m * forward_m
        )
        if length_m <= 1e-09:
            return None
        ratio = max(-1.0, min(1.0, forward_m / length_m))
        angle_deg = math.degrees(math.acos(ratio))
        effective_m = math.sqrt(right_m * right_m + down_m * down_m)
        return (angle_deg, effective_m, length_m, forward_m)

    def _baseline_camera_translation_in_side_rdf_m(
        self, baseline: RtkBaselineEstimate, side: str
    ) -> tuple[float, float, float] | None:
        extrinsic = self.inputs.latest_extrinsics.get().get(side)
        if extrinsic is None:
            return None
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        body_vector = world_ned_vector_to_body_frd(world_vector, extrinsic.q)
        camera_vector = body_frd_vector_to_camera(body_vector)
        scale_to_m = self.inputs.baseline_scale_m()
        if scale_to_m is None:
            return camera_vector
        length_m = math.sqrt(
            sum((component * component for component in camera_vector))
        )
        if length_m <= 1e-09:
            return None
        scale = float(scale_to_m) / length_m
        return (
            camera_vector[0] * scale,
            camera_vector[1] * scale,
            camera_vector[2] * scale,
        )

    def _baseline_forward_angle_level(self, angle_deg: float) -> str:
        if 80.0 <= angle_deg <= 100.0:
            return "good"
        if 70.0 <= angle_deg <= 110.0:
            return "warn"
        return "bad"

    def _optical_alignment_intrinsics_result(
        self,
    ) -> tuple[StereoIntrinsicsResult | None, str]:
        if self.inputs.active_intrinsics.get() is not None:
            return (self.inputs.active_intrinsics.get(), "active calibration result")
        if self.inputs.last_calibration_result.get() is not None:
            return (
                self.inputs.last_calibration_result.get(),
                "latest calibration result, not applied",
            )
        return (None, "intrinsics waiting")

    def _optical_alignment_mean_focal_px(self, axis: str) -> tuple[float | None, str]:
        result, source = self.actions.panel__optical_alignment_intrinsics_result.get(
            None
        )()
        if result is None:
            return (None, source)
        try:
            if axis == "x":
                left_value = float(result.left.camera_matrix[0][0])
                right_value = float(result.right.camera_matrix[0][0])
            else:
                left_value = float(result.left.camera_matrix[1][1])
                right_value = float(result.right.camera_matrix[1][1])
        except (TypeError, ValueError, IndexError):
            return (None, "intrinsics focal invalid")
        mean_value = (left_value + right_value) / 2.0
        if not isfinite(mean_value) or mean_value <= 0.0:
            return (None, "intrinsics focal invalid")
        return (mean_value, source)

    def _optical_alignment_expected_dy_px(self) -> tuple[float | None, str, str]:
        mode = str(self.inputs.optical_expected_dy_combo_currentData() or "rectified")
        if mode == "rectified":
            return (0.0, "Rectified 0 px", "good")
        if mode == "manual":
            return (self.inputs.optical_expected_dy_spin_value(), "Manual", "good")
        result, source = self.actions.panel__optical_alignment_intrinsics_result.get(
            None
        )()
        if result is None:
            return (None, source, "warn")
        try:
            left_cy = float(result.left.camera_matrix[1][2])
            right_cy = float(result.right.camera_matrix[1][2])
        except (TypeError, ValueError, IndexError):
            return (None, "intrinsics cy invalid", "bad")
        expected = left_cy - right_cy
        return (
            expected,
            f"Raw intrinsics cy diff ({source})",
            "warn" if result is not self.inputs.active_intrinsics.get() else "good",
        )

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
        if any((level == "bad" for level in levels)):
            return "bad"
        if any((level == "warn" for level in levels)):
            return "warn"
        if any((level == "good" for level in levels)):
            return "good"
        return "idle"

    def freeze_geometry_inputs(
        self,
        frozen: Mapping[str, object],
        *,
        statuses: tuple[MeasurementMethodStatus, ...] | None = None,
    ) -> FrozenGeometryInputs:
        """Capture scalar controls, calibration and method geometry exactly once.

        This is the input adapter. The pure resolver receives the returned DTO
        and never calls these scalar/configuration/optical input ports.
        """
        # Preserve preferred/fallback/hysteresis selection at the capture edge.
        # The resolver never chooses a method from a later live readiness list.
        selection_error = ""
        try:
            selected = (
                self.actions._select_measurement_method_for_attempt()
                if statuses is None
                else None
            )
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as exc:
            selected = None
            selection_error = str(exc)
        captured_statuses: tuple[MeasurementMethodStatus, ...] = ()
        resolved: dict[tuple[str, str], FrozenMethodGeometry] = {}
        errors: dict[tuple[str, str], str] = {}
        generation = str(
            read(self.capture_state._measurement_attempt_generation_id, "")
            or frozen["snapshot"].measurement_id
        )
        reason = str(read(self.methods_state._measurement_selection_reason, "selected"))
        advisories = tuple(read(self.capture_state._last_pair_advisories, ()) or ())
        selected_id = (
            selected.method_id
            if selected is not None
            else str(read(self.methods_state._measurement_selected_method_id, "") or "")
        )
        selected_instance = (
            selected.instance_id
            if selected is not None
            else str(
                read(self.methods_state._measurement_selected_instance_id, "") or ""
            )
        )
        try:
            if selection_error:
                raise ValueError(f"method selection capture failed: {selection_error}")
            captured_statuses = tuple(
                statuses
                if statuses is not None
                else self.actions._measurement_method_statuses()
            )
            max_delta = float(self.inputs._yolo_pair_max_delta_s.get()())
            max_vertical = float(self.inputs._yolo_vertical_max_delta_px.get()())
            config = GeometryConfigSnapshot(
                focal_length_px=float(self.inputs.focal_length_px()),
                max_time_delta_s=max_delta if math.isfinite(max_delta) else None,
                max_vertical_delta_px=(
                    max_vertical if math.isfinite(max_vertical) else None
                ),
                require_same_class=True,
                timestamp_basis=str(frozen["timestamp_basis"]),
                manual_focal_override=bool(self.inputs.manual_focal_override()),
                require_rtk_fixed_setting=bool(
                    self.inputs._workstation_setting.get()(
                        "require_rtk_fixed",
                        self.inputs.settings.get().require_rtk_fixed,
                    )
                ),
                field_quality_gates_advisory=True,
            )
            active_zoom = int(self.inputs.active_zoom())
            frozen_kd = bool(
                read(
                    self.methods_state._measurement_frozen_comparison_inputs_declared,
                    False,
                )
            )
            frozen_path = str(
                read(self.methods_state._pending_measurement_frozen_intrinsics_path, "")
                or ""
            )
            active_path = str(self.inputs._active_intrinsics_profile_path.get("") or "")
            intrinsics_zoom = self.inputs._active_intrinsics_source_zoom_level.get(None)
            extrinsics_zoom = self.inputs._active_extrinsics_source_zoom_level.get(None)
            for status in captured_statuses:
                key = (status.method_id, status.instance_id)
                try:
                    pose_evidence: dict[str, object] = {}
                    result = self._measurement_intrinsics_result_for_method(
                        status,
                        left_attitude=frozen.get("left_attitude"),
                        right_attitude=frozen.get("right_attitude"),
                        left_gps=frozen.get("left_gps"),
                        right_gps=frozen.get("right_gps"),
                        pose_evidence_out=pose_evidence,
                    )
                    nominal = (
                        result is not None
                        and dict(result.calibration_quality or {}).get("state")
                        == NOMINAL_INTRINSICS_QUALITY
                    )
                    operating = (
                        self.inputs.optical._operating_intrinsics_profile()
                        if nominal and not frozen_kd
                        else None
                    )
                    profile_path = (
                        frozen_path
                        if frozen_kd
                        else str(
                            getattr(operating, "source_path", "")
                            or active_path
                            or f"memory:active-calibration:zoom-{active_zoom:02d}"
                        )
                    )
                    intrinsics = (
                        None
                        if result is None
                        else StereoIntrinsicsSnapshot.from_result(
                            result,
                            active_zoom_level=active_zoom,
                            selected_profile_path=profile_path,
                            selected_extrinsics_profile_path=status.instance_id,
                            intrinsics_source_zoom_level=(
                                intrinsics_zoom
                                if operating is None
                                else operating.left.source_zoom_index
                            ),
                            extrinsics_source_zoom_level=extrinsics_zoom,
                            selection_mode=f"measurement_method/{status.method_id}",
                        )
                    )
                    yaw = None
                    yaw_ages: tuple[float | None, float | None] = (None, None)
                    if status.method_id == PIXHAWK_RP_GPS_BASELINE_YAW:
                        left_gps, right_gps = frozen.get("left_gps"), frozen.get(
                            "right_gps"
                        )
                        left_attitude, right_attitude = frozen.get(
                            "left_attitude"
                        ), frozen.get("right_attitude")
                        baseline = (
                            None
                            if left_gps is None or right_gps is None
                            else ned_delta_m(left_gps, right_gps)
                        )
                        if baseline is not None:
                            yaw = select_baseline_yaw(
                                north_m=float(baseline[0]),
                                east_m=float(baseline[1]),
                                p1_yaw_deg=getattr(left_attitude, "yaw_deg", None),
                                p2_yaw_deg=getattr(right_attitude, "yaw_deg", None),
                                reverse=self.actions._measurement_baseline_yaw_reverse(),
                            )
                        now_s = float(frozen["created_monotonic_s"])
                        yaw_ages = tuple(
                            (
                                None
                                if sample is None
                                else max(0.0, now_s - float(sample.timestamp_s))
                            )
                            for sample in (left_attitude, right_attitude)
                        )
                    resolved[key] = FrozenMethodGeometry(
                        intrinsics=intrinsics,
                        pose_evidence_json=(
                            json.dumps(
                                pose_evidence,
                                sort_keys=True,
                                ensure_ascii=False,
                                allow_nan=False,
                                separators=(",", ":"),
                            )
                            if pose_evidence
                            else ""
                        ),
                        nominal_intrinsics=nominal,
                        yaw_selection=yaw,
                        yaw_p1_age_s=yaw_ages[0],
                        yaw_p2_age_s=yaw_ages[1],
                    )
                except (
                    AttributeError,
                    KeyError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors[key] = f"frozen method input error: {exc}"
            return FrozenGeometryInputs(
                config,
                captured_statuses,
                resolved,
                errors,
                generation,
                reason,
                advisories,
                selected_method_id=selected_id,
                selected_instance_id=selected_instance,
            )
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as exc:
            return FrozenGeometryInputs(
                None,
                captured_statuses,
                resolved,
                errors,
                generation,
                reason,
                advisories,
                capture_error=f"geometry input capture failed: {exc}",
                selected_method_id=selected_id,
                selected_instance_id=selected_instance,
            )

    def _build_measurement_attempt(
        self,
        pair,
        *,
        method_status: MeasurementMethodStatus | None = None,
        frozen_inputs: Mapping[str, object] | None = None,
        use_cache: bool = True,
    ) -> MeasurementAttempt:
        frozen = frozen_inputs or self.actions._freeze_measurement_pair_inputs(pair)
        context = frozen.get("geometry_inputs")
        if not isinstance(context, FrozenGeometryInputs):
            # Legacy embedders may supply the pre-decomposition Mapping API.
            # Production capture always returns FrozenPairInputs with a context.
            context = self.freeze_geometry_inputs(
                frozen, statuses=None if method_status is None else (method_status,)
            )
        if method_status is None:
            method_status = next(
                (
                    status
                    for status in context.statuses
                    if status.ready
                    and status.method_id == context.selected_method_id
                    and status.instance_id == context.selected_instance_id
                ),
                None,
            )
            if method_status is None:
                raise ValueError("no ready measurement method at pair capture")
        attempt = resolve_measurement_attempt(frozen, method_status, context)
        key = attempt.detection_pair_key
        cached = (
            read(self.state._last_measurement_attempt, None)
            if use_cache and key == read(self.state._last_measurement_attempt_key, None)
            else None
        )
        if cached is not None:
            if cached.is_geometry_ready or not set(
                attempt.missing_geometry_inputs
            ) < set(cached.missing_geometry_inputs):
                return cached
        if use_cache:
            self.state._last_measurement_attempt_key = key
            self.state._last_measurement_attempt = attempt
        return attempt

    def _translation_right_in_left_m(
        self, attempt: MeasurementAttempt
    ) -> tuple[float, float, float] | None:
        """Return the immutable attempt translation without consulting latest state."""
        return attempt.translation_right_in_left_m

    def _latest_translation_right_in_left_m_for_diagnostics(
        self,
    ) -> tuple[float, float, float] | None:
        if self.inputs._distance_mode.get()() == "checkerboard":
            return self.actions.panel__checkerboard_translation_right_in_left_m.get(
                None
            )()
        left_ext = self.inputs.latest_extrinsics.get().get("left")
        if left_ext is None:
            return None
        baseline = self.inputs.latest_rtk_baseline.get()
        if baseline is None:
            return None
        usable, _reason = self.actions.panel__gps_baseline_usable_for_translation.get(
            None
        )(baseline)
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
        left_ext = self.inputs.latest_extrinsics.get().get("left")
        if left_ext is None:
            return None
        world_vector = (baseline.north_m, baseline.east_m, baseline.down_m)
        scale_to_m = self.inputs.baseline_scale_m()
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
        config = self.inputs.gnss_operation_manager.get().config
        if not config.readiness_gate_enabled:
            return None
        base_profile = self.inputs.gnss_fixed_base_profile.get()
        base_hash = "" if base_profile is None else str(base_profile.profile_hash)
        vector = self.inputs.latest_baseline_vector.get()
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
                json.dumps(
                    baseline_payload, sort_keys=True, separators=(",", ":")
                ).encode()
            ).hexdigest()
        routes = {
            side: f"mavlink:{side}"
            for side in self.inputs._gnss_base_rtcm_target_sides.get()()
        }
        selector = self.inputs._selected_rover_gps_state.get(None)
        if callable(selector):
            selected_samples = {
                side: selector(side) for side in ("left", "right", "drone")
            }
        else:
            pair_selector = self.inputs._selected_baseline_gps_states.get(None)
            if callable(pair_selector):
                selected_left, selected_right = pair_selector()
            else:
                selected_left = self.inputs.latest_gps.get().get("left")
                selected_right = self.inputs.latest_gps.get().get("right")
            selected_samples = {
                "left": selected_left,
                "right": selected_right,
                "drone": self.inputs.latest_gps.get().get("drone"),
            }
        device_ids = {
            side: str(getattr(sample, "topic", side))
            for (side, sample) in selected_samples.items()
            if sample is not None
        }
        return self.inputs.gnss_operation_manager.get().capture_measurement_contract(
            readiness,
            base_coordinate_profile_hash=base_hash,
            baseline_profile_hash=baseline_hash,
            device_ids=device_ids,
            route_endpoints=routes,
            datum="WGS84",
        )

    def _measurement_gnss_record_metadata(self) -> dict[str, object]:
        """Return the frozen GNSS contract fields for a recording row."""
        metadata = self.inputs.gnss_operation_manager.get().metadata()
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
            "gnss_contract_metadata": json.dumps(
                contract, sort_keys=True, separators=(",", ":")
            ),
            "gnss_readiness_timestamp_s": (
                ""
                if not isinstance(readiness, dict)
                else self.inputs._fmt_record_float.get()(
                    readiness.get("timestamp_s"), 6
                )
            ),
        }

    def _geometry_comparison_rows(self, pair) -> tuple[dict[str, object], ...]:
        """Preview F/R/D/Range for every extrinsics source, side by side.

        This never reads ``_distance_mode()`` and never touches
        ``active_intrinsics``/``selected_extrinsics_profile`` — it is a
        read-only comparison for the Measurement tab, independent of which
        source is officially selected for the recorded measurement.
        """
        optical_owner = self.inputs.optical
        intrinsics = self.inputs.active_intrinsics.get()
        profile_reader = getattr(
            optical_owner, "operating_intrinsics_profile_for_geometry", None
        )
        operating_profile = profile_reader() if callable(profile_reader) else None
        nominal = bool(
            operating_profile is not None
            and operating_profile.provenance.get("quality")
            == NOMINAL_INTRINSICS_QUALITY
        )
        if nominal:
            intrinsics = nominal_profile_to_stereo_result(operating_profile)
        elif callable(profile_reader) and operating_profile is None:
            return ()
        if intrinsics is None:
            return ()
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
            notes = tuple(
                filter(
                    None,
                    ("NOMINAL K,D (uncalibrated)" if nominal else "", provenance_note),
                )
            )
            suffix = " · " + " · ".join(notes) if notes else ""
            base = {
                "schema_version": limits.GEOMETRY_COMPARISON_ANALYSIS_SCHEMA,
                "evidence_namespace": limits.GEOMETRY_COMPARISON_ANALYSIS_NAMESPACE,
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
            rotation_right_to_left, translation_right_in_left_m = (
                invert_left_to_right_camera_transform(
                    rotation_left_to_right, translation_left_to_right_m
                )
            )
            return triangulate_pixels_with_relative_pose_attempt(
                left_pixel=left_pixel,
                right_pixel=right_pixel,
                left_intrinsics=intrinsics.left,
                right_intrinsics=intrinsics.right,
                rotation_right_to_left=rotation_right_to_left,
                translation_right_in_left_m=translation_right_in_left_m,
            )

        left_ext = self.inputs.latest_extrinsics.get().get("left")
        right_ext = self.inputs.latest_extrinsics.get().get("right")
        baseline = self.inputs.latest_rtk_baseline.get()
        pixhawk_attempt = None
        if left_ext is not None and baseline is not None:
            usable, _reason = (
                self.actions.panel__gps_baseline_usable_for_translation.get(None)(
                    baseline
                )
            )
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
                    right_body_to_world_q=(
                        right_ext.q if right_ext is not None else left_ext.q
                    ),
                    translation_right_in_left_m=translation_right_in_left_m,
                )
        add_row(
            "Pixhawk + RTK (live)", pixhawk_attempt, metadata={"source_state": "live"}
        )
        checkerboard_profile = self.inputs.checkerboard_extrinsics_profile.get(None)
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
                "source_state": (
                    "applied" if checkerboard_profile is not None else "missing"
                ),
                "source_profile_id": (
                    ""
                    if checkerboard_profile is None
                    else checkerboard_profile.profile_id
                ),
                "source_profile_hash": (
                    ""
                    if checkerboard_profile is None
                    else checkerboard_profile.content_sha256
                ),
            },
        )
        applied_profile = self.inputs.optical_pose_extrinsics_profile.get(None)
        applied_attempt = (
            None
            if applied_profile is None
            else relative_pose_attempt(
                applied_profile.extrinsics.rotation_left_to_right,
                applied_profile.extrinsics.translation_left_to_right_m,
            )
        )
        add_row(
            (
                "PnP active"
                if applied_profile is None
                else f"PnP active ({applied_profile.method})"
            ),
            applied_attempt,
            metadata={
                "source_state": "applied" if applied_profile is not None else "missing",
                "source_profile_id": (
                    "" if applied_profile is None else applied_profile.profile_id
                ),
                "source_profile_hash": (
                    "" if applied_profile is None else applied_profile.content_sha256
                ),
            },
        )
        candidate_poses = self.inputs.workstation_optical_pose_candidate_poses.get()()
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
                candidate_row["left_pose_ned"], candidate_row["right_pose_ned"]
            )
            attempt = (
                relative_pose_attempt(
                    [list(row) for row in stereo.rotation_right_from_left_rdf],
                    list(stereo.translation_right_from_left_rdf_m),
                )
                if candidate_applicable
                else None
            )
            optimized = tuple(candidate_row.get("optimized_dofs", ()))
            translation_observed = bool(
                set(optimized) & {"north_m", "east_m", "down_m"}
            )
            observed_text = ",".join(optimized) if optimized else "none"
            if translation_observed:
                note = f"observed={observed_text}"
            else:
                note = f"observed={observed_text} — Range/F/R/D는 prior 기준선 그대로 상속됨, 이 방식이 측정한 값 아님"
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
                    "candidate_profile_id": str(
                        candidate_row.get("candidate_id") or ""
                    ),
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

    def _optical_relative_rotation_fields(
        self, attempt_payload: Mapping[str, Any] | None = None
    ) -> dict[str, Any]:
        from Workstation.measurement.projection import optical_relative_rotation_fields

        try:
            accepted = self.inputs.optical._last_accepted_auto_suggestion
        except (AttributeError, RuntimeError):
            accepted = None
        return optical_relative_rotation_fields(
            attempt_payload, accepted=accepted, now_ns=time.time_ns()
        )
