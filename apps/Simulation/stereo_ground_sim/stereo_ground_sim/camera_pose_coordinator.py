"""Camera-oracle and operational pose-source coordination.

Contracts: Gazebo poses are World ENU; PX4 attitude is body-FRD relative to
Earth NED; OpenCV optical vectors are RDF; public stereo outputs use the
baseline-center FRD [forward, right, down] metre frame. Position and
orientation conversions remain explicit in PoseConversionAdapter.
"""

import hashlib
import json
import math
import time

from sasl_common.stereo_geometry import PinholeIntrinsics
from stereo_ground_sim.coordinate_frames import (
    apply_optical_rpy_error,
    apply_px4_attitude_rpy,
    build_baseline_center_frd,
    camera_optical_pose_from_quaternion,
    effective_optical_rpy_errors_deg,
    nominal_camera_poses,
    validate_rotation_matrix,
)
from stereo_ground_sim.gps_baseline import RawGpsFix, gps_baseline_enu_m
from stereo_ground_sim.pose_conversion_adapter import PoseConversionAdapter
from stereo_ground_sim.research_record_fields import camera_oracle_record_fields

_CAMERA_ORACLE_DEFAULT = object()

CAMERA_POSE_COORDINATION_METHODS = [
    "_configure_camera_frames",
    "_attitude_parameter_names",
    "_configure_attitude_error_state",
    "_ensure_camera_frames",
    "_finite_tuple",
    "_valid_sha256",
    "_parse_camera_pose_diagnostic",
    "_camera_pose_diagnostic_callback",
    "_camera_oracle_snapshot",
    "_gps_baseline_callback",
    "_px4_attitude_callback",
    "_legacy_attitude_sample",
    "_px4_attitude_camera_poses",
    "_global_baseline_callback",
    "_gps_fix_entry",
    "_gps_baseline_enu_m",
    "_pose_conversion_adapter",
    "_gps_baseline_camera_poses",
    "_uam_raw_gps_reference",
    "_auto_mission_phase_callback",
    "_recording_mission_phase",
    "_camera_oracle_record_fields",
    "_ensure_attitude_error_state",
    "_camera_intrinsics"
]


class CameraPoseCoordinator:
    def __init__(self, node, *, camera_oracle_unset, gps_baseline_anchor_policy, observed_pose_type, monotonic, gps_baseline):
        self.node = node
        self.camera_oracle_unset = camera_oracle_unset
        self.gps_baseline_anchor_policy = gps_baseline_anchor_policy
        self.observed_pose_type = observed_pose_type
        self.monotonic = monotonic
        self.gps_baseline = gps_baseline

    def _configure_camera_frames(self):
        """Freeze output frames and build the fixed PX4-biased nominal poses.

        ``nominal_left_camera``/``nominal_right_camera`` remain the zero-error
        optical poses used by the physical/ideal references.  The canonical
        Nominal (Raw) ray uses separate ``nominal_px4_*`` poses: the
        configured PX4 body-local attitude bias is applied once to those
        zero-error poses, while the Attitude ray continues to consume
        time-varying ``VehicleAttitude`` samples.
        """

        nominal_left, nominal_right = nominal_camera_poses(self.node.baseline_m, self.node.camera_height_m)
        self.node.nominal_left_camera = nominal_left
        self.node.nominal_right_camera = nominal_right
        self.node.baseline_center_frd = build_baseline_center_frd(
            nominal_left.center_world_enu_m,
            nominal_right.center_world_enu_m,
            nominal_left.forward_world_enu,
            nominal_right.forward_world_enu,
        )
        self.node.desired_left_camera = apply_optical_rpy_error(
            nominal_left,
            math.radians(self.node.effective_left_optical_roll_error_deg),
            math.radians(self.node.effective_left_optical_pitch_error_deg),
            math.radians(self.node.effective_left_optical_yaw_error_deg),
        )
        self.node.desired_right_camera = apply_optical_rpy_error(
            nominal_right,
            math.radians(self.node.effective_right_optical_roll_error_deg),
            math.radians(self.node.effective_right_optical_pitch_error_deg),
            math.radians(self.node.effective_right_optical_yaw_error_deg),
        )
        self.node.nominal_px4_left_camera = apply_px4_attitude_rpy(
            nominal_left,
            math.radians(float(getattr(self.node, "cam_l_px4_attitude_roll_deg", 0.0))),
            math.radians(float(getattr(self.node, "cam_l_px4_attitude_pitch_deg", 0.0))),
            math.radians(float(getattr(self.node, "cam_l_px4_attitude_yaw_deg", 0.0))),
        )
        self.node.nominal_px4_right_camera = apply_px4_attitude_rpy(
            nominal_right,
            math.radians(float(getattr(self.node, "cam_r_px4_attitude_roll_deg", 0.0))),
            math.radians(float(getattr(self.node, "cam_r_px4_attitude_pitch_deg", 0.0))),
            math.radians(float(getattr(self.node, "cam_r_px4_attitude_yaw_deg", 0.0))),
        )

    @staticmethod
    def _attitude_parameter_names():
        return (
            "cam_l_optical_roll_deg",
            "cam_l_optical_pitch_deg",
            "cam_l_optical_yaw_deg",
            "cam_r_optical_roll_deg",
            "cam_r_optical_pitch_deg",
            "cam_r_optical_yaw_deg",
            "cam_l_px4_attitude_roll_deg",
            "cam_l_px4_attitude_pitch_deg",
            "cam_l_px4_attitude_yaw_deg",
            "cam_r_px4_attitude_roll_deg",
            "cam_r_px4_attitude_pitch_deg",
            "cam_r_px4_attitude_yaw_deg",
            "common_optical_roll_error_deg",
            "relative_optical_roll_error_deg",
            "common_optical_pitch_error_deg",
            "relative_optical_pitch_error_deg",
            "common_optical_yaw_error_deg",
            "relative_optical_yaw_error_deg",
        )

    def _configure_attitude_error_state(self):
        left, right = effective_optical_rpy_errors_deg(
            (
                self.node.cam_l_optical_roll_deg,
                self.node.cam_l_optical_pitch_deg,
                self.node.cam_l_optical_yaw_deg,
            ),
            (
                self.node.cam_r_optical_roll_deg,
                self.node.cam_r_optical_pitch_deg,
                self.node.cam_r_optical_yaw_deg,
            ),
            (
                self.node.common_optical_roll_error_deg,
                self.node.common_optical_pitch_error_deg,
                self.node.common_optical_yaw_error_deg,
            ),
            (
                self.node.relative_optical_roll_error_deg,
                self.node.relative_optical_pitch_error_deg,
                self.node.relative_optical_yaw_error_deg,
            ),
        )
        (
            self.node.effective_left_optical_roll_error_deg,
            self.node.effective_left_optical_pitch_error_deg,
            self.node.effective_left_optical_yaw_error_deg,
        ) = left
        (
            self.node.effective_right_optical_roll_error_deg,
            self.node.effective_right_optical_pitch_error_deg,
            self.node.effective_right_optical_yaw_error_deg,
        ) = right
        self.node.attitude_error_frame = "camera_optical_rdf"
        self.node.attitude_angle_unit = "deg"
        self.node.attitude_parameterization = "direct_plus_common_relative"
        self.node.attitude_rotation_composition = (
            "R_Cnominal_Cactual=R_Down(yaw)@R_Right(pitch)@R_Forward(roll)"
        )

    def _ensure_camera_frames(self):
        self.node._ensure_attitude_error_state()
        if hasattr(self.node, "baseline_center_frd"):
            return
        self.node.baseline_m = float(getattr(self.node, "baseline_m", 25.0))
        self.node.camera_height_m = float(getattr(self.node, "camera_height_m", 5.0))

        self.node._configure_camera_frames()

    @staticmethod
    def _finite_tuple(value, *, size, field):
        if not isinstance(value, (list, tuple)) or len(value) != size:
            raise ValueError(f"{field} must contain exactly {size} values")
        parsed = tuple(float(item) for item in value)
        if not all(math.isfinite(item) for item in parsed):
            raise ValueError(f"{field} must be finite")
        return parsed

    @staticmethod
    def _valid_sha256(value):
        return (
            isinstance(value, str)
            and len(value) == 64
            and all(character in "0123456789abcdef" for character in value)
        )

    def _parse_camera_pose_diagnostic(self, camera_label, encoded):
        try:
            payload = json.loads(str(encoded))
        except (TypeError, json.JSONDecodeError) as exc:
            raise ValueError("camera oracle diagnostic is not valid JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("camera oracle diagnostic must be a JSON object")
        if payload.get("schema") != "camera_pose_comparison/v1":
            raise ValueError("camera oracle diagnostic schema is unsupported")
        if payload.get("camera_label") != camera_label:
            raise ValueError("camera oracle diagnostic camera label does not match topic")
        expected_hash = getattr(self.node, "camera_oracle_generated_sdf_sha256", "")
        received_hash = payload.get("generated_sdf_sha256")
        if not self.node._valid_sha256(expected_hash):
            raise ValueError("camera oracle expected generated SDF SHA-256 is unavailable")
        if received_hash != expected_hash:
            raise ValueError("camera oracle generated SDF SHA-256 does not match launch world")
        expected_model = f"{camera_label}_stereo_camera"
        if payload.get("observed_base_link_entity_name") != f"{expected_model}::base_link":
            raise ValueError("camera oracle base-link entity does not match expected rig")
        if payload.get("observed_optical_entity_name") != f"{expected_model}::optical_link":
            raise ValueError("camera oracle optical entity does not match expected rig")
        base_id = payload.get("observed_base_link_entity_id")
        optical_id = payload.get("observed_optical_entity_id")
        if any(not isinstance(value, int) or isinstance(value, bool) for value in (base_id, optical_id)):
            raise ValueError("camera oracle entity IDs must be integers")
        stamp_ns = payload.get("gazebo_stamp_ns")
        if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or stamp_ns <= 0:
            raise ValueError("camera oracle Gazebo stamp must be a positive integer")
        position = self.node._finite_tuple(
            payload.get("observed_optical_position_world_enu_m"),
            size=3,
            field="observed_optical_position_world_enu_m",
        )
        orientation = self.node._finite_tuple(
            payload.get("observed_optical_orientation_world_from_entity_xyzw"),
            size=4,
            field="observed_optical_orientation_world_from_entity_xyzw",
        )
        pose = camera_optical_pose_from_quaternion(position, orientation)
        return self.observed_pose_type(
            camera_label=camera_label,
            pose=pose,
            gazebo_stamp_ns=stamp_ns,
            generated_sdf_sha256=received_hash,
            base_entity_name=payload["observed_base_link_entity_name"],
            base_entity_id=base_id,
            optical_entity_name=payload["observed_optical_entity_name"],
            optical_entity_id=optical_id,
            optical_position_world_enu_m=position,
            optical_orientation_world_from_entity_xyzw=orientation,
            simulation_epoch=getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0),
            received_monotonic_s=self.monotonic(),
        )

    def _camera_pose_diagnostic_callback(self, camera_label, msg):
        try:
            observation = self.node._parse_camera_pose_diagnostic(camera_label, msg.data)
        except (TypeError, ValueError) as exc:
            condition = getattr(self.node, "yolo_condition", None)
            if condition is None:
                setattr(self.node, f"observed_{camera_label}_camera", None)
                self.node.camera_oracle_pair = (
                    None if camera_label == "left" else getattr(self.node, "observed_left_camera", None),
                    None if camera_label == "right" else getattr(self.node, "observed_right_camera", None),
                )
                self.node.camera_oracle_rejection_count = getattr(self.node, "camera_oracle_rejection_count", 0) + 1
                self.node.last_camera_oracle_rejection_reason = str(exc)
            else:
                with condition:
                    setattr(self.node, f"observed_{camera_label}_camera", None)
                    self.node.camera_oracle_pair = (
                        None if camera_label == "left" else self.node.observed_left_camera,
                        None if camera_label == "right" else self.node.observed_right_camera,
                    )
                    self.node.camera_oracle_rejection_count += 1
                    self.node.last_camera_oracle_rejection_reason = str(exc)
            get_logger = getattr(self.node, "get_logger", None)
            if get_logger is not None:
                get_logger().warning(f"Rejected {camera_label} camera oracle diagnostic: {exc}")
            return False
        condition = getattr(self.node, "yolo_condition", None)
        if condition is None:
            setattr(self.node, f"observed_{camera_label}_camera", observation)
            self.node.camera_oracle_pair = (
                observation if camera_label == "left" else getattr(self.node, "observed_left_camera", None),
                observation if camera_label == "right" else getattr(self.node, "observed_right_camera", None),
            )
        else:
            with condition:
                setattr(self.node, f"observed_{camera_label}_camera", observation)
                self.node.camera_oracle_pair = (
                    observation if camera_label == "left" else self.node.observed_left_camera,
                    observation if camera_label == "right" else self.node.observed_right_camera,
                )
                self.node.last_camera_oracle_rejection_reason = ""
        return True

    def _camera_oracle_snapshot(self):
        pair = getattr(self.node, "camera_oracle_pair", None)
        if pair is None:
            left = getattr(self.node, "observed_left_camera", None)
            right = getattr(self.node, "observed_right_camera", None)
        else:
            left, right = pair
        if left is None or right is None:
            return None
        current_epoch = getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0)
        max_age_s = float(getattr(self.node, "camera_oracle_max_age_s", 2.5))
        now = self.monotonic()
        if (
            left.simulation_epoch != current_epoch
            or right.simulation_epoch != current_epoch
            or now - left.received_monotonic_s > max_age_s
            or now - right.received_monotonic_s > max_age_s
        ):
            return None
        return left, right

    def _gps_baseline_callback(self, camera_label, message):
        """Retain the newest valid raw GPS fix for baseline-only comparison."""
        with self.node.lock:
            provider = self.node._ensure_pose_source_provider_locked()
            provider.store_raw_gps(
                camera_label,
                message,
                epoch=getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0),
                simulation_timestamp_ns=getattr(
                    getattr(self.node, "simulation_epoch", None), "last_clock_stamp_ns", None
                ),
                received_ns=time.monotonic_ns(),
            )

    def _px4_attitude_callback(self, camera_label, message):
        """Store one validated PX4 VehicleAttitude (Hamilton WXYZ, FRD->NED)."""
        with self.node.lock:
            self.node._ensure_pose_source_provider_locked().store_attitude(
                camera_label,
                message,
                epoch=getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0),
                simulation_timestamp_ns=getattr(
                    getattr(self.node, "simulation_epoch", None), "last_clock_stamp_ns", None
                ),
                received_ns=time.monotonic_ns(),
            )

    @staticmethod
    def _legacy_attitude_sample(sample):
        return PoseConversionAdapter._legacy_attitude_sample(sample)

    def _px4_attitude_camera_poses(
        self,
        camera_sources,
        *,
        include_provenance=False,
        operational_snapshot=None,
        observation_timestamp_ns=None,
    ):
        return self.node._pose_conversion_adapter()._px4_attitude_camera_poses(
            camera_sources,
            include_provenance=include_provenance,
            operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
        )
    def _global_baseline_callback(self, camera_label, message):
        """Retain the newest valid PX4 fused-global-position fix."""
        with self.node.lock:
            self.node._ensure_pose_source_provider_locked().store_global_position(
                camera_label,
                message,
                epoch=getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0),
                simulation_timestamp_ns=getattr(
                    getattr(self.node, "simulation_epoch", None), "last_clock_stamp_ns", None
                ),
                received_ns=time.monotonic_ns(),
            )

    @staticmethod
    def _gps_fix_entry(entry, current_epoch):
        return PoseConversionAdapter._gps_fix_entry(entry, current_epoch)

    def _gps_baseline_enu_m(self, left_fix, right_fix):
        """Compatibility seam for deterministic baseline-source tests."""
        return self.gps_baseline(left_fix, right_fix)

    def _pose_conversion_adapter(self):
        adapter = getattr(self.node, "_pose_conversion_adapter_owner", None)
        if adapter is None:
            adapter = PoseConversionAdapter(
                self.node,
                gps_baseline_anchor_policy=self.gps_baseline_anchor_policy,
            )
            self.node._pose_conversion_adapter_owner = adapter
        return adapter

    def _gps_baseline_camera_poses(
        self,
        source,
        *,
        operational_snapshot=None,
        observation_timestamp_ns=None,
    ):
        return self.node._pose_conversion_adapter()._gps_baseline_camera_poses(
            source,
            operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
        )

    def _uam_raw_gps_reference(self):
        return self.node._pose_conversion_adapter()._uam_raw_gps_reference()
    def _auto_mission_phase_callback(self, msg):
        phase = str(getattr(msg, "data", "")).strip()
        if phase:
            self.node.external_mission_phase = phase

    def _recording_mission_phase(self, fallback=None):
        phase = str(getattr(self.node, "external_mission_phase", "")).strip()
        if phase:
            return phase
        if fallback is not None:
            return fallback
        if not hasattr(self.node, "start_time"):
            return "unavailable"
        return self.node._mission_state()["phase"]

    def _camera_oracle_record_fields(self, snapshot=_CAMERA_ORACLE_DEFAULT):
        if snapshot is _CAMERA_ORACLE_DEFAULT or snapshot is self.camera_oracle_unset:
            snapshot = self.node._camera_oracle_snapshot()
        return camera_oracle_record_fields(
            snapshot,
            expected_sdf_sha256=getattr(
                self.node, "camera_oracle_generated_sdf_sha256", ""
            ),
        )

    def _ensure_attitude_error_state(self):
        for name in self.node._attitude_parameter_names():
            setattr(self.node, name, float(getattr(self.node, name, 0.0)))
        self.node._configure_attitude_error_state()

    def _camera_intrinsics(self, width, height):
        width = float(width)
        height = float(height)
        fx = (width * 0.5) / math.tan(float(self.node.horizontal_fov_rad) * 0.5)
        return PinholeIntrinsics(fx_px=fx, fy_px=fx, cx_px=width * 0.5, cy_px=height * 0.5)
