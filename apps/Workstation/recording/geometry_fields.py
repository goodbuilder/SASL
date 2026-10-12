"""Project attitude, GPS, baseline, and session-reference coordinate fields.

GPS position is NED; attitude is PX4 FRD-to-NED quaternion wxyz; camera translation
is left-optical RDF. The reference owner retains first-observed session truth."""
from __future__ import annotations
import json
import math
import time
from Workstation.workstation_core.attitude_geometry import quaternion_conjugate, quaternion_multiply, relative_attitude, relative_rotation_angle_deg
from Workstation.workstation_core.measurement_attempt import AttitudeSnapshot, GpsSnapshot, MeasurementAttempt
from Workstation.workstation_core.measurement_methods import CHECKERBOARD_STEREO
from Workstation.workstation_core.coordinate_contract import BODY_FRAME, CAMERA_FRAME, CAMERA_MOUNT_PROFILE, WORLD_FRAME
from Workstation.workstation_core.rtk_geometry import ned_delta_m, project_ned_baseline_to_p1_body_frd
from Workstation.workstation_core.sync import ATTITUDE_STALE_AGE_S, ATTITUDE_STALE_PAIR_DELTA_S, ATTITUDE_TIMING_POLICY, ATTITUDE_VERY_STALE_AGE_S, ATTITUDE_VERY_STALE_PAIR_DELTA_S, classify_attitude_timing
from .state import (RecordingReferenceState)
from .ports import GeometryFieldsPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .calibration_fields import CalibrationFields
    from .formatting import Formatting


class GeometryFields:
    def __init__(self, *,
        references: RecordingReferenceState,
        boundary: GeometryFieldsPort,
    ) -> None:
        self.references = references
        self.boundary = boundary

    def bind_collaborators(self, *,
        calibration_fields: "CalibrationFields",
        formatting: "Formatting",
    ) -> None:
        self.calibration_fields = calibration_fields
        self.formatting = formatting

    def _record_reference_delta_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {
            "gps_position_frame": "session_left_start_gps_antenna_ned",
            "gps_position_axis_contract": "north,east,down; metres",
            "attitude_orientation_contract": "px4_frd_to_ned; quaternion_wxyz",
        }
        attitudes = {
            "left": measurement_attempt.left_attitude,
            "right": measurement_attempt.right_attitude,
        }
        gps_states = {
            "left": measurement_attempt.left_gps,
            "right": measurement_attempt.right_gps,
        }
        for camera, attitude in attitudes.items():
            if attitude is None:
                continue
            reference = self.references.recording_reference_attitudes.setdefault(camera, attitude)
            delta = relative_attitude(reference.quaternion_wxyz, attitude.quaternion_wxyz)
            prefix = f"{camera}_pixhawk"
            row.update(
                {
                    f"{prefix}_reference_q_wxyz": json.dumps(
                        list(reference.quaternion_wxyz), separators=(",", ":")
                    ),
                    f"{prefix}_delta_roll_deg": self.formatting._fmt_record_float(delta.roll_deg, 9),
                    f"{prefix}_delta_pitch_deg": self.formatting._fmt_record_float(delta.pitch_deg, 9),
                    f"{prefix}_delta_yaw_deg": self.formatting._fmt_record_float(delta.yaw_deg, 9),
                    f"{prefix}_delta_rotation_angle_deg": self.formatting._fmt_record_float(
                        relative_rotation_angle_deg(reference.quaternion_wxyz, attitude.quaternion_wxyz), 9
                    ),
                }
            )

        if (
            attitudes["left"] is not None
            and attitudes["right"] is not None
            and "left" in self.references.recording_reference_attitudes
            and "right" in self.references.recording_reference_attitudes
        ):
            initial_relative_q = quaternion_multiply(
                quaternion_conjugate(self.references.recording_reference_attitudes["left"].quaternion_wxyz),
                self.references.recording_reference_attitudes["right"].quaternion_wxyz,
            )
            current_relative_q = quaternion_multiply(
                quaternion_conjugate(attitudes["left"].quaternion_wxyz),
                attitudes["right"].quaternion_wxyz,
            )
            row["left_right_relative_reference_q_wxyz"] = json.dumps(
                list(initial_relative_q), separators=(",", ":")
            )
            row["left_right_relative_q_wxyz"] = json.dumps(
                list(current_relative_q), separators=(",", ":")
            )
            row["left_right_relative_delta_rotation_angle_deg"] = self.formatting._fmt_record_float(
                relative_rotation_angle_deg(initial_relative_q, current_relative_q), 9
            )

        if self.references.recording_origin_gps is None and gps_states["left"] is not None:
            self.references.recording_origin_gps = gps_states["left"]
        for camera, gps in gps_states.items():
            if gps is None:
                continue
            reference = self.references.recording_reference_gps.setdefault(camera, gps)
            origin_delta = (
                None if self.references.recording_origin_gps is None else ned_delta_m(self.references.recording_origin_gps, gps)
            )
            reference_delta = ned_delta_m(reference, gps)
            prefix = f"{camera}_gps_antenna"
            if origin_delta is not None:
                row[f"{prefix}_north_m"] = self.formatting._fmt_record_float(origin_delta[0], 9)
                row[f"{prefix}_east_m"] = self.formatting._fmt_record_float(origin_delta[1], 9)
                row[f"{prefix}_down_m"] = self.formatting._fmt_record_float(origin_delta[2], 9)
            if reference_delta is not None:
                row[f"{prefix}_delta_north_m"] = self.formatting._fmt_record_float(reference_delta[0], 9)
                row[f"{prefix}_delta_east_m"] = self.formatting._fmt_record_float(reference_delta[1], 9)
                row[f"{prefix}_delta_down_m"] = self.formatting._fmt_record_float(reference_delta[2], 9)
                row[f"{prefix}_delta_position_m"] = self.formatting._fmt_record_float(
                    math.sqrt(sum(value * value for value in reference_delta)), 9
                )

        if gps_states["left"] is not None and gps_states["right"] is not None:
            baseline_ned = ned_delta_m(gps_states["left"], gps_states["right"])
            if baseline_ned is not None:
                if self.references.recording_reference_baseline_ned_m is None:
                    self.references.recording_reference_baseline_ned_m = baseline_ned
                baseline_delta = tuple(
                    value - reference
                    for value, reference in zip(baseline_ned, self.references.recording_reference_baseline_ned_m)
                )
                baseline_length_m = math.sqrt(sum(value * value for value in baseline_ned))
                reference_baseline_length_m = math.sqrt(
                    sum(value * value for value in self.references.recording_reference_baseline_ned_m)
                )
                row.update(
                    {
                        "baseline_north_m": self.formatting._fmt_record_float(baseline_ned[0], 9),
                        "baseline_east_m": self.formatting._fmt_record_float(baseline_ned[1], 9),
                        "baseline_down_m": self.formatting._fmt_record_float(baseline_ned[2], 9),
                        "baseline_length_m": self.formatting._fmt_record_float(baseline_length_m, 9),
                        "baseline_reference_length_m": self.formatting._fmt_record_float(
                            reference_baseline_length_m, 9
                        ),
                        "baseline_length_change_m": self.formatting._fmt_record_float(
                            baseline_length_m - reference_baseline_length_m, 9
                        ),
                        "baseline_delta_north_m": self.formatting._fmt_record_float(baseline_delta[0], 9),
                        "baseline_delta_east_m": self.formatting._fmt_record_float(baseline_delta[1], 9),
                        "baseline_delta_down_m": self.formatting._fmt_record_float(baseline_delta[2], 9),
                        "baseline_delta_length_m": self.formatting._fmt_record_float(
                            math.sqrt(sum(value * value for value in baseline_delta)), 9
                        ),
                    }
                )
        return row

    def _record_external_parameters(
        self, *, estimate=None, measurement_attempt: MeasurementAttempt
    ) -> dict[str, str]:
        triangulation_source = measurement_attempt.triangulation_source
        checkerboard_mode = measurement_attempt.measurement_method_id == CHECKERBOARD_STEREO
        baseline_yaw_mode = bool(measurement_attempt.measurement_yaw_source)
        relative_pose_mode = measurement_attempt.measurement_geometry_kind == "relative_pose"
        component_sources = dict(measurement_attempt.measurement_component_sources)
        uses_live_gnss = "live_gnss" in component_sources.values()
        uses_live_pixhawk = "live_pixhawk" in component_sources.values()
        attitude_timing_status = classify_attitude_timing(
            uses_pixhawk_attitude=uses_live_pixhawk,
            left_age_s=(
                measurement_attempt.left_attitude_age_s
                if estimate is None
                else estimate.left_attitude_age_s
            ),
            right_age_s=(
                measurement_attempt.right_attitude_age_s
                if estimate is None
                else estimate.right_attitude_age_s
            ),
            pair_delta_s=(
                measurement_attempt.attitude_pair_delta_s
                if estimate is None
                else estimate.attitude_pair_delta_s
            ),
        )
        checkerboard = self.calibration_fields._record_checkerboard_extrinsics_parameters(measurement_attempt)
        row: dict[str, str] = {
            "triangulation_extrinsics_source": triangulation_source,
            "measurement_method_id": measurement_attempt.measurement_method_id,
            "measurement_method_instance_id": measurement_attempt.measurement_method_instance_id,
            "measurement_geometry_kind": measurement_attempt.measurement_geometry_kind,
            "measurement_geometry_sha256": measurement_attempt.measurement_geometry_sha256,
            "measurement_selection_reason": measurement_attempt.measurement_selection_reason,
            "measurement_component_sources": json.dumps(
                component_sources, sort_keys=True, separators=(",", ":")
            ),
            "use_rtk_baseline": self.formatting._fmt_record_bool(uses_live_gnss),
            "rtk_baseline_applied": self.formatting._fmt_record_bool(uses_live_gnss),
            "require_rtk_fixed": self.formatting._fmt_record_bool(
                measurement_attempt.geometry_config.require_rtk_fixed_setting
                if uses_live_gnss
                else False
            ),
            "use_triangulation": self.formatting._fmt_record_bool(True),
            "distance_mode": measurement_attempt.measurement_method_id,
            "world_frame": WORLD_FRAME,
            "body_frame": BODY_FRAME,
            "camera_frame": CAMERA_FRAME,
            "camera_mount_profile": CAMERA_MOUNT_PROFILE,
            "mount_assumption_model": (
                "checkerboard-stereo-extrinsics-v1"
                if checkerboard_mode
                else "gps-rtk-baseline-perpendicular-yaw-v1"
                if baseline_yaw_mode
                else "hybrid-candidate-live-components-v1"
                if relative_pose_mode and (uses_live_pixhawk or uses_live_gnss)
                else "optical-pose-relative-r-t-v1"
                if relative_pose_mode
                else "gps-rtk-baseline-only-v1"
            ),
            "mount_assumption_status": (
                "checkerboard stereo extrinsics"
                if checkerboard_mode
                else "GPS/RTK baseline perpendicular-yaw assumption"
                if baseline_yaw_mode
                else "resolved candidate/live component geometry"
                if relative_pose_mode
                else "GPS/RTK baseline only"
            ),
            "mount_assumption_detail": (
                "applied_checkerboard_stereo_extrinsics_used_as_fixed_optical_transform"
                if checkerboard_mode
                else "GPS/RTK baseline azimuth +/-90deg with Pixhawk R/P and selected branch"
                if baseline_yaw_mode
                else "resolved_relative_pose_with_per_dof_sources"
                if relative_pose_mode
                else "live_gps_rtk_baseline_used_as_stereo_translation"
            ),
            "checkerboard_extrinsics_active": self.formatting._fmt_record_bool(
                checkerboard_mode and bool(checkerboard["active"])
            ),
            "checkerboard_extrinsics_pair_count": (
                str(checkerboard.get("pair_count", "")) if checkerboard_mode else ""
            ),
            "checkerboard_extrinsics_reprojection_error_px": self.formatting._fmt_record_float(
                checkerboard.get("reprojection_error_px") if checkerboard_mode else None, 6
            ),
            "checkerboard_extrinsics_frame_contract": str(checkerboard["frame_contract"]),
            "require_attitude": self.formatting._fmt_record_bool(uses_live_pixhawk),
            "attitude_used_for_triangulation": self.formatting._fmt_record_bool(uses_live_pixhawk),
            "attitude_timing_policy": ATTITUDE_TIMING_POLICY,
            "attitude_timing_status": attitude_timing_status,
            "attitude_timing_blocks_measurement": self.formatting._fmt_record_bool(uses_live_pixhawk),
            "attitude_stale_age_threshold_s": self.formatting._fmt_record_float(ATTITUDE_STALE_AGE_S, 3),
            "attitude_very_stale_age_threshold_s": self.formatting._fmt_record_float(
                ATTITUDE_VERY_STALE_AGE_S, 3
            ),
            "attitude_stale_pair_delta_threshold_s": self.formatting._fmt_record_float(
                ATTITUDE_STALE_PAIR_DELTA_S, 3
            ),
            "attitude_very_stale_pair_delta_threshold_s": self.formatting._fmt_record_float(
                ATTITUDE_VERY_STALE_PAIR_DELTA_S, 3
            ),
            "left_attitude_age_s": self.formatting._fmt_record_float(
                measurement_attempt.left_attitude_age_s if estimate is None else estimate.left_attitude_age_s, 6
            ),
            "right_attitude_age_s": self.formatting._fmt_record_float(
                measurement_attempt.right_attitude_age_s if estimate is None else estimate.right_attitude_age_s, 6
            ),
            "attitude_pair_delta_s": self.formatting._fmt_record_float(
                measurement_attempt.attitude_pair_delta_s if estimate is None else estimate.attitude_pair_delta_s, 6
            ),
            "relative_roll_deg": self.formatting._fmt_record_float(
                measurement_attempt.relative_roll_deg if estimate is None else estimate.relative_roll_deg, 6
            ),
            "relative_pitch_deg": self.formatting._fmt_record_float(
                measurement_attempt.relative_pitch_deg if estimate is None else estimate.relative_pitch_deg, 6
            ),
            "relative_yaw_deg": self.formatting._fmt_record_float(
                measurement_attempt.relative_yaw_deg if estimate is None else estimate.relative_yaw_deg, 6
            ),
        }
        baseline = measurement_attempt.baseline
        if baseline is not None:
            camera_translation = baseline.translation_right_in_left_rdf_m
            baseline_forward_geometry = self.boundary._baseline_forward_geometry_from_camera_translation(camera_translation)
            ned = baseline.ned_right_from_left_m
            row.update(
                {
                    "rtk_baseline_north_m": self.formatting._fmt_record_float(None if ned is None else ned[0], 6),
                    "rtk_baseline_east_m": self.formatting._fmt_record_float(None if ned is None else ned[1], 6),
                    "rtk_baseline_down_m": self.formatting._fmt_record_float(None if ned is None else ned[2], 6),
                    "baseline_p1_body_frd_state": "WAITING",
                    "baseline_p1_body_frd_forward_m": "",
                    "baseline_p1_body_frd_right_m": "",
                    "baseline_p1_body_frd_down_m": "",
                    "baseline_p1_body_frd_attitude_age_s": "",
                    "baseline_p1_body_frd_frame": BODY_FRAME,
                    "baseline_p1_body_frd_axis_contract": (
                        "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres"
                    ),
                    "rtk_left_age_s": self.formatting._fmt_record_float(baseline.left_age_at_capture_s, 6),
                    "rtk_right_age_s": self.formatting._fmt_record_float(baseline.right_age_at_capture_s, 6),
                    "rtk_fix_quality": baseline.fix_quality,
                    "rtk_warnings": "; ".join(baseline.warnings),
                    "selected_gps_pair_delta_s": self.formatting._fmt_record_float(baseline.gps_pair_delta_s, 6),
                    "selected_left_gps_detection_age_s": self.formatting._fmt_record_float(
                        baseline.left_age_from_detection_midpoint_s, 6
                    ),
                    "selected_right_gps_detection_age_s": self.formatting._fmt_record_float(
                        baseline.right_age_from_detection_midpoint_s, 6
                    ),
                }
            )
            left_attitude = measurement_attempt.left_attitude
            if ned is not None and left_attitude is not None:
                projection = project_ned_baseline_to_p1_body_frd(
                    ned,
                    left_attitude.quaternion_wxyz,
                    attitude_timestamp_s=left_attitude.app_receive_monotonic_s,
                    attitude_age_s=measurement_attempt.left_attitude_age_s,
                )
                row.update(
                    {
                        "baseline_p1_body_frd_state": "READY",
                        "baseline_p1_body_frd_forward_m": self.formatting._fmt_record_float(
                            projection.forward_m, 6
                        ),
                        "baseline_p1_body_frd_right_m": self.formatting._fmt_record_float(
                            projection.right_m, 6
                        ),
                        "baseline_p1_body_frd_down_m": self.formatting._fmt_record_float(
                            projection.down_m, 6
                        ),
                        "baseline_p1_body_frd_attitude_age_s": self.formatting._fmt_record_float(
                            projection.attitude_age_s, 6
                        ),
                    }
                )
            if camera_translation is not None:
                row.update(
                    {
                        "baseline_left_camera_rdf_right_m": self.formatting._fmt_record_float(camera_translation[0], 6),
                        "baseline_left_camera_rdf_down_m": self.formatting._fmt_record_float(camera_translation[1], 6),
                        "baseline_left_camera_rdf_forward_m": self.formatting._fmt_record_float(camera_translation[2], 6),
                    }
                )
            if baseline_forward_geometry is not None:
                angle_deg, effective_m, length_m, forward_m = baseline_forward_geometry
                row.update(
                    {
                        "baseline_rdf_forward_angle_deg": self.formatting._fmt_record_float(angle_deg, 6),
                        "baseline_effective_perpendicular_m": self.formatting._fmt_record_float(effective_m, 6),
                        "baseline_camera_frame_length_m": self.formatting._fmt_record_float(length_m, 6),
                        "baseline_forward_component_m": self.formatting._fmt_record_float(forward_m, 6),
                    }
                )
        for camera in ("left", "right"):
            attitude = (
                measurement_attempt.left_attitude
                if camera == "left"
                else measurement_attempt.right_attitude
            )
            attitude_age_s = (
                measurement_attempt.left_attitude_age_s
                if camera == "left"
                else measurement_attempt.right_attitude_age_s
            )
            gps = measurement_attempt.left_gps if camera == "left" else measurement_attempt.right_gps
            gps_age_s = None
            if measurement_attempt.baseline is not None:
                gps_age_s = (
                    measurement_attempt.baseline.left_age_at_capture_s
                    if camera == "left"
                    else measurement_attempt.baseline.right_age_at_capture_s
                )
            if gps_age_s is None and gps is not None:
                frame_timestamp_s = (
                    measurement_attempt.left_detection.app_receive_monotonic_s
                    if camera == "left"
                    else measurement_attempt.right_detection.app_receive_monotonic_s
                )
                gps_age_s = abs(frame_timestamp_s - gps.app_receive_monotonic_s)
            row.update(self._record_extrinsic_state(camera, attitude, attitude_age_s))
            row.update(self._record_gps_state(camera, gps, gps_age_s))
            row.update(self._record_gnss_relative_state(camera))
        return row

    def _record_extrinsic_state(
        self,
        camera: str,
        state: AttitudeSnapshot | None,
        age_s: float | None,
    ) -> dict[str, str]:
        prefix = f"{camera}_pixhawk"
        if state is None:
            return {}
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_roll_deg": self.formatting._fmt_record_float(state.roll_deg, 6),
            f"{prefix}_pitch_deg": self.formatting._fmt_record_float(state.pitch_deg, 6),
            f"{prefix}_yaw_deg": self.formatting._fmt_record_float(state.yaw_deg, 6),
            f"{prefix}_q": json.dumps(list(state.quaternion_wxyz), separators=(",", ":")),
            f"{prefix}_q_w": self.formatting._fmt_record_float(state.quaternion_wxyz[0], 9),
            f"{prefix}_q_x": self.formatting._fmt_record_float(state.quaternion_wxyz[1], 9),
            f"{prefix}_q_y": self.formatting._fmt_record_float(state.quaternion_wxyz[2], 9),
            f"{prefix}_q_z": self.formatting._fmt_record_float(state.quaternion_wxyz[3], 9),
            f"{prefix}_age_s": self.formatting._fmt_record_float(age_s, 6),
            f"{prefix}_app_receive_timestamp_s": self.formatting._fmt_record_float(state.timestamp_s, 6),
            f"{prefix}_app_receive_monotonic_ns": int(round(state.timestamp_s * 1_000_000_000.0)),
            f"{prefix}_px4_timestamp_s": self.formatting._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_delta_q_reset_wxyz": (
                "" if state.delta_q_reset is None else json.dumps(list(state.delta_q_reset), separators=(",", ":"))
            ),
            f"{prefix}_quat_reset_counter": (
                "" if state.quat_reset_counter is None else state.quat_reset_counter
            ),
            f"{prefix}_mavlink_source_system": (
                "" if state.mavlink_source_system is None else state.mavlink_source_system
            ),
            f"{prefix}_mavlink_source_component": (
                "" if state.mavlink_source_component is None else state.mavlink_source_component
            ),
            f"{prefix}_mavlink_sequence": "" if state.mavlink_sequence is None else state.mavlink_sequence,
            f"{prefix}_mavlink_message_id": (
                "" if state.mavlink_message_id is None else state.mavlink_message_id
            ),
        }

    def _record_gps_state(
        self,
        camera: str,
        state: GpsSnapshot | None,
        age_s: float | None,
    ) -> dict[str, str]:
        prefix = f"{camera}_gps"
        if state is None:
            return {}
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_lat_deg": self.formatting._fmt_record_float(state.latitude_deg, 9),
            f"{prefix}_lon_deg": self.formatting._fmt_record_float(state.longitude_deg, 9),
            f"{prefix}_alt_m": self.formatting._fmt_record_float(state.altitude_m, 6),
            f"{prefix}_altitude_ellipsoid_m": self.formatting._fmt_record_float(state.altitude_ellipsoid_m, 6),
            f"{prefix}_fix_type": "" if state.fix_type is None else str(state.fix_type),
            f"{prefix}_satellites": "" if state.satellites_used is None else str(state.satellites_used),
            f"{prefix}_eph_m": self.formatting._fmt_record_float(state.eph_m, 6),
            f"{prefix}_epv_m": self.formatting._fmt_record_float(state.epv_m, 6),
            f"{prefix}_heading_deg": self.formatting._fmt_record_float(state.heading_deg, 6),
            f"{prefix}_heading_accuracy_deg": self.formatting._fmt_record_float(state.heading_accuracy_deg, 6),
            f"{prefix}_rtk_label": state.rtk_label,
            f"{prefix}_age_s": self.formatting._fmt_record_float(age_s, 6),
            f"{prefix}_app_receive_timestamp_s": self.formatting._fmt_record_float(state.timestamp_s, 6),
            f"{prefix}_app_receive_monotonic_ns": int(round(state.timestamp_s * 1_000_000_000.0)),
            f"{prefix}_px4_timestamp_s": self.formatting._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
            f"{prefix}_gps_utc_valid": self.formatting._fmt_record_bool(state.gps_utc_us is not None),
            f"{prefix}_timestamp_time_relative_us": (
                ""
                if state.gps_timestamp_time_relative_us is None
                else state.gps_timestamp_time_relative_us
            ),
            f"{prefix}_velocity_n_m_s": self.formatting._fmt_record_float(state.velocity_n_m_s, 6),
            f"{prefix}_velocity_e_m_s": self.formatting._fmt_record_float(state.velocity_e_m_s, 6),
            f"{prefix}_velocity_d_m_s": self.formatting._fmt_record_float(state.velocity_d_m_s, 6),
            f"{prefix}_velocity_ned_valid": self.formatting._fmt_record_bool(state.velocity_ned_valid),
            f"{prefix}_hdop": self.formatting._fmt_record_float(state.hdop, 6),
            f"{prefix}_vdop": self.formatting._fmt_record_float(state.vdop, 6),
            f"{prefix}_device_id": "" if state.device_id is None else state.device_id,
            f"{prefix}_time_source": state.time_source,
            f"{prefix}_mavlink_source_system": (
                "" if state.mavlink_source_system is None else state.mavlink_source_system
            ),
            f"{prefix}_mavlink_source_component": (
                "" if state.mavlink_source_component is None else state.mavlink_source_component
            ),
            f"{prefix}_mavlink_sequence": "" if state.mavlink_sequence is None else state.mavlink_sequence,
            f"{prefix}_mavlink_message_id": (
                "" if state.mavlink_message_id is None else state.mavlink_message_id
            ),
        }

    def _record_gnss_relative_state(self, camera: str) -> dict[str, str]:
        state = self.boundary.latest_gnss_relative.get(camera)
        prefix = f"{camera}_gnss_relative"
        if state is None:
            return {}
        age_s = max(0.0, time.monotonic() - state.timestamp_s)
        heading_deg = None if state.heading_rad is None else math.degrees(state.heading_rad)
        heading_accuracy_deg = None if state.heading_accuracy_rad is None else math.degrees(state.heading_accuracy_rad)
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_device_id": "" if state.device_id is None else str(state.device_id),
            f"{prefix}_heading_deg": self.formatting._fmt_record_float(heading_deg, 6),
            f"{prefix}_heading_accuracy_deg": self.formatting._fmt_record_float(heading_accuracy_deg, 6),
            f"{prefix}_position_length_m": self.formatting._fmt_record_float(state.position_length_m, 6),
            f"{prefix}_accuracy_length_m": self.formatting._fmt_record_float(state.accuracy_length_m, 6),
            f"{prefix}_gnss_fix_ok": self.formatting._fmt_record_bool(state.gnss_fix_ok),
            f"{prefix}_differential_solution": self.formatting._fmt_record_bool(state.differential_solution),
            f"{prefix}_relative_position_valid": self.formatting._fmt_record_bool(state.relative_position_valid),
            f"{prefix}_carrier_solution_floating": self.formatting._fmt_record_bool(state.carrier_solution_floating),
            f"{prefix}_carrier_solution_fixed": self.formatting._fmt_record_bool(state.carrier_solution_fixed),
            f"{prefix}_moving_base_mode": self.formatting._fmt_record_bool(state.moving_base_mode),
            f"{prefix}_heading_valid": self.formatting._fmt_record_bool(state.heading_valid),
            f"{prefix}_age_s": self.formatting._fmt_record_float(age_s, 6),
            f"{prefix}_px4_timestamp_s": self.formatting._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
            f"{prefix}_gps_utc_valid": self.formatting._fmt_record_bool(state.gps_utc_us is not None),
            f"{prefix}_mavlink_source_system": (
                "" if state.mavlink_source_system is None else state.mavlink_source_system
            ),
            f"{prefix}_mavlink_source_component": (
                "" if state.mavlink_source_component is None else state.mavlink_source_component
            ),
            f"{prefix}_mavlink_sequence": "" if state.mavlink_sequence is None else state.mavlink_sequence,
            f"{prefix}_mavlink_message_id": (
                "" if state.mavlink_message_id is None else state.mavlink_message_id
            ),
        }
