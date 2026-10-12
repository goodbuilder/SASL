"""Pure geometry assembly from frozen pair/configuration/calibration inputs.

No Qt, callbacks, live telemetry or controller state is reachable here. Existing
hash serialization, RDF transform direction and baseline-center FRD output stay
unchanged. Geometry snapshots carry the calibration used for this exact pair.
"""

from __future__ import annotations
import hashlib
import json
from collections.abc import Mapping
from Workstation.measurement.frozen_inputs import FrozenGeometryInputs
from Workstation.workstation_core.attitude_geometry import relative_attitude
from Workstation.workstation_core.baseline_yaw import (
    BASELINE_YAW_MINUS_90_DEG,
    BaselineYawSelection,
    quaternion_from_roll_pitch_yaw_deg,
)
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
    build_checkerboard_baseline_snapshot,
    build_pixhawk_baseline_snapshot,
    detection_pair_key,
)
from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    PIXHAWK_RTK_LIVE,
    PIXHAWK_RP_GPS_BASELINE_YAW,
    MeasurementMethodStatus,
)
from Workstation.workstation_core.method_evaluation import FrozenMeasurementPairSnapshot


def measurement_geometry_hash(
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
            None
            if geometry_right_quaternion is None
            else list(geometry_right_quaternion)
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


def resolve_measurement_attempt(
    frozen: Mapping[str, object],
    method_status: MeasurementMethodStatus,
    context: FrozenGeometryInputs,
) -> MeasurementAttempt:
    """Resolve one method only from a previously frozen shared pair."""
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
        else "live_pixhawk_mount" if pixhawk_geometry_method else "relative_pose"
    )
    resolved = context.for_method(method_status)
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
    geometry_left_quaternion_alternate = None
    geometry_right_quaternion_alternate = None
    alternate_yaw_deg: float | None = None
    if baseline_yaw_method:
        yaw_selection, yaw_p1_age_s, yaw_p2_age_s = (
            resolved.yaw_selection,
            resolved.yaw_p1_age_s,
            resolved.yaw_p2_age_s,
        )
        if yaw_selection is None or not yaw_selection.ready:
            raise ValueError(
                "baseline yaw selection unavailable: "
                + (
                    "no GPS baseline or Pixhawk yaw reference"
                    if yaw_selection is None
                    else yaw_selection.reason
                )
            )
        if (
            left_attitude is None
            or right_attitude is None
            or yaw_selection.selected_yaw_deg is None
        ):
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
        alternate_yaw_deg = (
            yaw_selection.candidate_plus90_deg
            if yaw_selection.selected_offset_deg == BASELINE_YAW_MINUS_90_DEG
            else yaw_selection.candidate_minus90_deg
        )
        geometry_left_quaternion_alternate = quaternion_from_roll_pitch_yaw_deg(
            left_attitude.roll_deg, left_attitude.pitch_deg, alternate_yaw_deg
        )
        geometry_right_quaternion_alternate = quaternion_from_roll_pitch_yaw_deg(
            right_attitude.roll_deg, right_attitude.pitch_deg, alternate_yaw_deg
        )
    relative = (
        None
        if geometry_left_quaternion is None or geometry_right_quaternion is None
        else relative_attitude(geometry_left_quaternion, geometry_right_quaternion)
    )
    intrinsics = resolved.intrinsics
    pose_evidence = (
        json.loads(resolved.pose_evidence_json) if resolved.pose_evidence_json else {}
    )
    warnings: tuple[str, ...] = tuple(
        dict.fromkeys((*method_status.warnings, *context.pair_advisories))
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
    if resolved.nominal_intrinsics:
        warnings = tuple(
            dict.fromkeys(
                (
                    *warnings,
                    "NOMINAL_UNVERIFIED intrinsics; zero distortion assumed; precision not qualified",
                )
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
            warnings = tuple(
                dict.fromkeys(
                    (*warnings, "selected method R,t unresolved for this pair")
                )
            )
        elif baseline.warnings:
            warnings = tuple(dict.fromkeys((*warnings, *baseline.warnings)))
    else:
        baseline = None
        if left_gps is not None and right_gps is not None:
            baseline = build_pixhawk_baseline_snapshot(
                left_gps=left_gps,
                right_gps=right_gps,
                selected_left_quaternion_wxyz=geometry_left_quaternion
                or (None if left_attitude is None else left_attitude.quaternion_wxyz),
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
            warnings = tuple(
                dict.fromkeys((*warnings, "waiting for live GPS baseline"))
            )
    geometry_sha256 = measurement_geometry_hash(
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
        left=left_detection,
        right=right_detection,
        left_pairing_timestamp_s=left_pairing_timestamp_s,
        right_pairing_timestamp_s=right_pairing_timestamp_s,
    )
    key = (
        *detection_key,
        "geometry_revision_v2",
        method_status.instance_id,
        geometry_sha256,
    )
    geometry_config = context.config
    if geometry_config is None:
        raise ValueError("geometry configuration unavailable at pair capture")
    candidate_attempt_id = snapshot.pair_sequence
    attempt = MeasurementAttempt(
        attempt_id=candidate_attempt_id,
        attempt_session_id=str(context.generation_id or snapshot.measurement_id),
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
            "GOOD"
            if baseline is None or baseline.fix_quality == "NOT_USED"
            else baseline.fix_quality
        ),
        warnings=warnings,
        measurement_method_id=method_id,
        measurement_method_instance_id=method_status.instance_id,
        measurement_geometry_kind=geometry_kind,
        measurement_geometry_sha256=geometry_sha256,
        measurement_selection_reason=context.selection_reason,
        measurement_component_sources=method_status.component_sources,
        measurement_pose_evidence_json=(
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
        measurement_started_monotonic_s=measurement_start,
        requested_timestamp_basis=str(
            frozen.get("requested_timestamp_basis") or timestamp_basis
        ),
        timing_fallback_reason=str(frozen.get("timing_fallback_reason") or ""),
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
        measurement_yaw_candidate_minus90_deg=(
            None if yaw_selection is None else yaw_selection.candidate_minus90_deg
        ),
        measurement_yaw_candidate_plus90_deg=(
            None if yaw_selection is None else yaw_selection.candidate_plus90_deg
        ),
        measurement_yaw_alternate_deg=alternate_yaw_deg,
        measurement_yaw_p1_error_minus90_deg=(
            None if yaw_selection is None else yaw_selection.p1_error_minus90_deg
        ),
        measurement_yaw_p1_error_plus90_deg=(
            None if yaw_selection is None else yaw_selection.p1_error_plus90_deg
        ),
        measurement_yaw_p2_error_minus90_deg=(
            None if yaw_selection is None else yaw_selection.p2_error_minus90_deg
        ),
        measurement_yaw_p2_error_plus90_deg=(
            None if yaw_selection is None else yaw_selection.p2_error_plus90_deg
        ),
        geometry_left_quaternion_wxyz_alternate=geometry_left_quaternion_alternate,
        geometry_right_quaternion_wxyz_alternate=geometry_right_quaternion_alternate,
    )
    return attempt
