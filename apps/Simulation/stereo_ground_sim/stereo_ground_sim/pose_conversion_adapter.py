"""PX4/GPS source validation and camera-pose conversion adapter.

Contracts:
- PX4 attitude samples are Hamilton WXYZ rotations body FRD -> earth NED.
- raw/fused GPS fixes are WGS84 geodetic with MSL altitude.
- GPS relative baselines are world ENU metres.
- camera pose outputs use world ENU centres and OpenCV optical RDF basis.
- UAM public point outputs use baseline-center FRD [Forward, Right, Down] metres.
"""

import math
import time
from dataclasses import replace

from stereo_ground_sim.coordinate_frames import (
    camera_optical_pose_from_px4_attitude_wxyz,
    relative_rotation_angle_deg,
)
from stereo_ground_sim.gps_baseline import (
    declination_from_gps_baseline_rad,
    gps_relative_enu_m,
    norm_m,
)
from stereo_ground_sim.pose_source_provider import Px4AttitudeSample


class PoseConversionAdapter:
    def __init__(self, node, *, gps_baseline_anchor_policy):
        self.node = node
        self.gps_baseline_anchor_policy = gps_baseline_anchor_policy

    @staticmethod
    def _legacy_attitude_sample(sample):
        """Read pre-v4 tuple state retained by deterministic compatibility tests."""

        if sample is None or isinstance(sample, Px4AttitudeSample):
            return sample
        quaternion, timestamp_us, reset_counter, epoch, received = sample
        received_ns = (
            int(received)
            if float(received) > 1.0e12
            else int(float(received) * 1_000_000_000)
        )
        try:
            quaternion = tuple(float(value) for value in quaternion)
            valid = len(quaternion) == 4 and all(
                math.isfinite(value) for value in quaternion
            ) and math.sqrt(sum(value * value for value in quaternion)) > 1.0e-15
            if valid:
                magnitude = math.sqrt(sum(value * value for value in quaternion))
                quaternion = tuple(value / magnitude for value in quaternion)
        except (TypeError, ValueError):
            valid = False
            quaternion = None
        return Px4AttitudeSample(
            quaternion_wxyz=quaternion,
            timestamp_us=int(timestamp_us),
            simulation_timestamp_ns=None,
            reset_counter=int(reset_counter),
            simulation_epoch=int(epoch),
            received_monotonic_ns=received_ns,
            valid=valid,
            failure_reason="" if valid else "invalid_px4_attitude",
        )

    def _px4_attitude_camera_poses(
        self,
        camera_sources,
        *,
        include_provenance=False,
        operational_snapshot=None,
        observation_timestamp_ns=None,
        declination_correction_rad=0.0,
    ):
        def finish(poses, status, provenance):
            return (
                (poses, status, provenance)
                if include_provenance
                else (poses, status)
            )

        provenance = {
            "pixel_source": "observed",
            "attitude_source": "px4_vehicle_attitude",
            "px4_quaternion_order": "wxyz",
            "px4_attitude_semantics": "body_frd_to_earth_ned",
            "left_attitude_topic": getattr(self.node, "px4_attitude_left_topic", ""),
            "right_attitude_topic": getattr(self.node, "px4_attitude_right_topic", ""),
            "simulation_epoch": getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0),
            "operational_source_snapshot_timestamp_ns": (
                None
                if not isinstance(operational_snapshot, dict)
                else operational_snapshot.get("captured_simulation_timestamp_ns")
            ),
            "attitude_alignment_method": (
                "latest_valid_px4_sample_at_image_pair_acceptance"
                if isinstance(operational_snapshot, dict)
                else "latest_valid_px4_sample_at_computation"
            ),
        }
        if camera_sources is None:
            return finish(None, "missing_camera_oracle", provenance)
        snapshot_attitudes = (
            operational_snapshot.get("px4_attitudes")
            if isinstance(operational_snapshot, dict)
            else None
        )
        if isinstance(snapshot_attitudes, dict):
            left_sample = self.node._legacy_attitude_sample(snapshot_attitudes.get("left"))
            right_sample = self.node._legacy_attitude_sample(snapshot_attitudes.get("right"))
        else:
            with self.node.lock:
                left_sample = self.node._legacy_attitude_sample(
                    self.node.px4_attitudes.get("left")
                )
                right_sample = self.node._legacy_attitude_sample(
                    self.node.px4_attitudes.get("right")
                )
        if left_sample is None:
            return finish(None, "missing_left_attitude", provenance)
        if right_sample is None:
            return finish(None, "missing_right_attitude", provenance)
        for label, sample in (("left", left_sample), ("right", right_sample)):
            quaternion = sample.quaternion_wxyz
            provenance.update(
                {
                    f"{label}_attitude_timestamp_us": sample.timestamp_us,
                    f"{label}_attitude_simulation_timestamp_ns": sample.simulation_timestamp_ns,
                    f"{label}_attitude_receive_monotonic_ns": sample.received_monotonic_ns,
                    f"{label}_attitude_reset_counter": sample.reset_counter,
                    **{
                        f"{label}_px4_q_{axis}": (
                            None if quaternion is None else quaternion[index]
                        )
                        for index, axis in enumerate(("w", "x", "y", "z"))
                    },
                }
            )
            if not sample.valid:
                return finish(
                    None,
                    sample.failure_reason or f"invalid_{label}_quaternion",
                    provenance,
                )
        epoch = getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0)
        if left_sample.simulation_epoch != epoch or right_sample.simulation_epoch != epoch:
            return finish(None, "attitude_epoch_mismatch", provenance)
        now_ns = (
            int(operational_snapshot.get("captured_monotonic_ns"))
            if isinstance(operational_snapshot, dict)
            and operational_snapshot.get("captured_monotonic_ns") is not None
            else time.monotonic_ns()
        )
        left_age_ms = (now_ns - left_sample.received_monotonic_ns) / 1_000_000.0
        right_age_ms = (now_ns - right_sample.received_monotonic_ns) / 1_000_000.0
        pair_skew_ms = abs(left_sample.timestamp_us - right_sample.timestamp_us) / 1_000.0
        provenance.update(
            {
                "left_attitude_age_ms": left_age_ms,
                "right_attitude_age_ms": right_age_ms,
                "attitude_pair_skew_ms": pair_skew_ms,
            }
        )
        if left_age_ms > self.node.px4_attitude_max_age_s * 1_000.0:
            return finish(None, "stale_left_attitude", provenance)
        if right_age_ms > self.node.px4_attitude_max_age_s * 1_000.0:
            return finish(None, "stale_right_attitude", provenance)
        if pair_skew_ms > self.node.px4_attitude_pair_skew_s * 1_000.0:
            return finish(None, "attitude_pair_skew_exceeded", provenance)
        if observation_timestamp_ns is not None:
            for label, sample in (("left", left_sample), ("right", right_sample)):
                sample_stamp_ns = sample.simulation_timestamp_ns
                if sample_stamp_ns is None:
                    return finish(
                        None,
                        f"missing_{label}_attitude_simulation_timestamp",
                        provenance,
                    )
                alignment_dt_s = (
                    int(sample_stamp_ns) - int(observation_timestamp_ns)
                ) / 1_000_000_000.0
                provenance[f"{label}_attitude_alignment_dt_s"] = alignment_dt_s
                if abs(alignment_dt_s) > self.node.px4_attitude_max_age_s:
                    return finish(
                        None,
                        f"{label}_attitude_observation_skew_exceeded",
                        provenance,
                    )
        try:
            left_source, right_source = camera_sources
            left_pose = getattr(left_source, "pose", left_source)
            right_pose = getattr(right_source, "pose", right_source)
            poses = (
                camera_optical_pose_from_px4_attitude_wxyz(
                    left_pose.center_world_enu_m,
                    left_sample.quaternion_wxyz,
                    declination_correction_rad=declination_correction_rad,
                ),
                camera_optical_pose_from_px4_attitude_wxyz(
                    right_pose.center_world_enu_m,
                    right_sample.quaternion_wxyz,
                    declination_correction_rad=declination_correction_rad,
                ),
            )
        except ValueError:
            return finish(None, "invalid_px4_attitude", provenance)
        try:
            provenance["left_attitude_truth_residual_deg"] = relative_rotation_angle_deg(
                left_pose, poses[0]
            )
            provenance["right_attitude_truth_residual_deg"] = relative_rotation_angle_deg(
                right_pose, poses[1]
            )
        except ValueError:
            provenance["left_attitude_truth_residual_deg"] = None
            provenance["right_attitude_truth_residual_deg"] = None
        return finish(poses, "available", provenance)

    def _px4_attitude_true_north_camera_poses(
        self,
        camera_sources,
        *,
        include_provenance=False,
        operational_snapshot=None,
        observation_timestamp_ns=None,
    ):
        """Attitude-ray poses with a live GPS-baseline declination correction.

        Estimates local magnetic declination in-situ from the existing
        left/right camera GPS baseline (see ``declination_from_gps_baseline_rad``)
        and applies it to the same PX4 VehicleAttitude pipeline used by the
        plain Attitude ray, so this Ray reflects a true-north-referenced
        heading instead of the magnetometer's uncorrected one. Fails closed
        (returns ``None`` poses) if GPS is unavailable/stale or the
        declination estimate is degenerate -- it never silently falls back
        to an uncorrected pose.
        """

        def finish(poses, status, provenance):
            return (
                (poses, status, provenance)
                if include_provenance
                else (poses, status)
            )

        uncorrected_poses, attitude_status, attitude_provenance = self._px4_attitude_camera_poses(
            camera_sources,
            include_provenance=True,
            operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
        )
        provenance = dict(attitude_provenance)
        provenance["camera_orientation_source"] = "px4_vehicle_attitude_declination_corrected"
        if uncorrected_poses is None:
            return finish(None, attitude_status, provenance)

        current_epoch = getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0)
        snapshot_fixes = (
            operational_snapshot.get("gps_baseline_fixes")
            if isinstance(operational_snapshot, dict)
            else None
        )
        if isinstance(snapshot_fixes, dict):
            left_entry = snapshot_fixes.get("left")
            right_entry = snapshot_fixes.get("right")
        else:
            with self.node.lock:
                left_entry = self.node.gps_baseline_fixes.get("left")
                right_entry = self.node.gps_baseline_fixes.get("right")
        if left_entry is None:
            return finish(None, "missing_left_gps_fix", provenance)
        if right_entry is None:
            return finish(None, "missing_right_gps_fix", provenance)
        left_fix, left_received_ns, left_epoch, _left_ts_ns = self.node._gps_fix_entry(
            left_entry, current_epoch
        )
        right_fix, right_received_ns, right_epoch, _right_ts_ns = self.node._gps_fix_entry(
            right_entry, current_epoch
        )
        if left_epoch != current_epoch or right_epoch != current_epoch:
            return finish(None, "gps_epoch_mismatch", provenance)
        now_ns = (
            int(operational_snapshot.get("captured_monotonic_ns"))
            if isinstance(operational_snapshot, dict)
            and operational_snapshot.get("captured_monotonic_ns") is not None
            else time.monotonic_ns()
        )
        left_age_ms = (now_ns - left_received_ns) / 1_000_000.0
        right_age_ms = (now_ns - right_received_ns) / 1_000_000.0
        provenance.update(
            {"gps_left_age_ms": left_age_ms, "gps_right_age_ms": right_age_ms}
        )
        if left_age_ms > self.node.gps_baseline_max_age_s * 1_000.0:
            return finish(None, "stale_left_gps_fix", provenance)
        if right_age_ms > self.node.gps_baseline_max_age_s * 1_000.0:
            return finish(None, "stale_right_gps_fix", provenance)

        nominal_baseline = tuple(
            self.node.nominal_right_camera.center_world_enu_m[index]
            - self.node.nominal_left_camera.center_world_enu_m[index]
            for index in range(3)
        )
        declination_rad = declination_from_gps_baseline_rad(
            left_fix,
            right_fix,
            nominal_baseline,
            self.node.nominal_left_camera,
            uncorrected_poses[0],
        )
        if declination_rad is None:
            return finish(None, "declination_estimate_unavailable", provenance)
        provenance["declination_correction_deg"] = math.degrees(declination_rad)

        corrected_poses, corrected_status, corrected_provenance = self._px4_attitude_camera_poses(
            camera_sources,
            include_provenance=True,
            operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
            declination_correction_rad=declination_rad,
        )
        if corrected_poses is None:
            return finish(None, corrected_status, provenance)
        provenance.update(corrected_provenance)
        provenance["camera_orientation_source"] = "px4_vehicle_attitude_declination_corrected"
        provenance["declination_correction_deg"] = math.degrees(declination_rad)
        return finish(corrected_poses, "available", provenance)

    @staticmethod
    def _gps_fix_entry(entry, current_epoch):
        if entry is None:
            return None
        if len(entry) == 2:
            fix, received = entry
            received_ns = (
                int(received)
                if float(received) > 1.0e12
                else int(float(received) * 1_000_000_000)
            )
            return fix, received_ns, int(current_epoch), None
        if len(entry) == 3:
            fix, received_ns, epoch = entry
            return fix, int(received_ns), int(epoch), None
        fix, received_ns, epoch, simulation_timestamp_ns = entry
        return (
            fix,
            int(received_ns),
            int(epoch),
            None if simulation_timestamp_ns is None else int(simulation_timestamp_ns),
        )

    def _gps_baseline_camera_poses(
        self,
        source,
        *,
        operational_snapshot=None,
        observation_timestamp_ns=None,
    ):
        """Build configured-optical-attitude poses separated by one PX4 position source.

        Orientation comes from ``desired_left_camera``/``desired_right_camera``
        (the CAM_*_OPTICAL_* configured attitude), not the level nominal pose,
        so the ray reflects the camera's actual look direction. Only the
        camera *centers* are replaced with the GPS/global-position-derived
        separation, which is what isolates raw position error from the
        time-varying PX4 VehicleAttitude stream used by the Attitude ray.
        """

        provenance = {
            "pixel_source": "observed",
            "baseline_source": "px4_raw_sensor_gps" if source == "gps_baseline" else "px4_vehicle_global_position",
            "baseline_anchor_policy": self.gps_baseline_anchor_policy,
            "left_camera_center_source": "nominal_left_optical_center_anchor",
            "right_camera_center_source": "nominal_left_anchor_plus_px4_relative_enu_separation",
            "camera_orientation_source": "configured_cam_optical_rpy",
            "left_gps_topic": getattr(self.node, "gps_baseline_left_topic", ""),
            "right_gps_topic": getattr(self.node, "gps_baseline_right_topic", ""),
            "operational_source_snapshot_timestamp_ns": (
                None
                if not isinstance(operational_snapshot, dict)
                else operational_snapshot.get("captured_simulation_timestamp_ns")
            ),
            "position_alignment_method": (
                "latest_valid_px4_position_at_image_pair_acceptance"
                if isinstance(operational_snapshot, dict)
                else "latest_valid_px4_position_at_computation"
            ),
        }
        if not getattr(self.node, "enable_gps_baseline_triangulation", False):
            return None, "disabled", provenance
        source = self.node._validate_gps_baseline_source(source)
        if source == "both":
            raise ValueError("both is an output selection, not one baseline source")
        snapshot_key = (
            "gps_baseline_fixes"
            if source == "gps_baseline"
            else "global_baseline_fixes"
        )
        snapshot_fixes = (
            operational_snapshot.get(snapshot_key)
            if isinstance(operational_snapshot, dict)
            else None
        )
        if isinstance(snapshot_fixes, dict):
            left_entry = snapshot_fixes.get("left")
            right_entry = snapshot_fixes.get("right")
            failures = dict(getattr(self.node, "gps_baseline_failures", {}))
        else:
            fixes = (
                self.node.gps_baseline_fixes
                if source == "gps_baseline"
                else self.node.global_baseline_fixes
            )
            with self.node.lock:
                left_entry = fixes.get("left")
                right_entry = fixes.get("right")
                failures = dict(getattr(self.node, "gps_baseline_failures", {}))
        current_epoch = getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0)
        if left_entry is None:
            failure = failures.get("left") if source == "gps_baseline" else None
            status = (
                failure[0]
                if failure is not None and failure[1] == current_epoch
                else "missing_left_gps_fix"
            )
            return None, status, provenance
        if right_entry is None:
            failure = failures.get("right") if source == "gps_baseline" else None
            status = (
                failure[0]
                if failure is not None and failure[1] == current_epoch
                else "missing_right_gps_fix"
            )
            return None, status, provenance
        left_fix, left_received_ns, left_epoch, left_simulation_timestamp_ns = self.node._gps_fix_entry(
            left_entry, current_epoch
        )
        right_fix, right_received_ns, right_epoch, right_simulation_timestamp_ns = self.node._gps_fix_entry(
            right_entry, current_epoch
        )
        provenance.update(
            {
                "gps_left_timestamp_us": left_fix.timestamp_us,
                "gps_right_timestamp_us": right_fix.timestamp_us,
                "left_gps_simulation_timestamp_ns": left_simulation_timestamp_ns,
                "right_gps_simulation_timestamp_ns": right_simulation_timestamp_ns,
                "simulation_epoch": current_epoch,
            }
        )
        if left_epoch != current_epoch or right_epoch != current_epoch:
            return None, "gps_epoch_mismatch", provenance
        now_ns = (
            int(operational_snapshot.get("captured_monotonic_ns"))
            if isinstance(operational_snapshot, dict)
            and operational_snapshot.get("captured_monotonic_ns") is not None
            else time.monotonic_ns()
        )
        left_age_ms = (now_ns - left_received_ns) / 1_000_000.0
        right_age_ms = (now_ns - right_received_ns) / 1_000_000.0
        pair_skew_ms = abs(left_fix.timestamp_us - right_fix.timestamp_us) / 1_000.0
        provenance.update(
            {
                "left_gps_age_ms": left_age_ms,
                "right_gps_age_ms": right_age_ms,
                "gps_pair_skew_ms": pair_skew_ms,
            }
        )
        if left_age_ms > self.node.gps_baseline_max_age_s * 1_000.0:
            return None, "stale_left_gps_fix", provenance
        if right_age_ms > self.node.gps_baseline_max_age_s * 1_000.0:
            return None, "stale_right_gps_fix", provenance
        if pair_skew_ms > self.node.gps_baseline_max_age_s * 1_000.0:
            return None, "gps_pair_skew_exceeded", provenance
        if observation_timestamp_ns is not None:
            for label, sample_stamp_ns in (
                ("left", left_simulation_timestamp_ns),
                ("right", right_simulation_timestamp_ns),
            ):
                if sample_stamp_ns is None:
                    return None, f"missing_{label}_gps_simulation_timestamp", provenance
                alignment_dt_s = (
                    int(sample_stamp_ns) - int(observation_timestamp_ns)
                ) / 1_000_000_000.0
                provenance[f"{label}_gps_alignment_dt_s"] = alignment_dt_s
                if abs(alignment_dt_s) > self.node.gps_baseline_max_age_s:
                    return (
                        None,
                        f"{label}_gps_observation_skew_exceeded",
                        provenance,
                    )
        try:
            baseline_enu = self.node._gps_baseline_enu_m(left_fix, right_fix)
        except ValueError:
            return None, "invalid_raw_gps_fix", provenance
        if not all(math.isfinite(value) for value in baseline_enu):
            return None, "invalid_raw_gps_baseline", provenance
        baseline_length_m = norm_m(baseline_enu)
        if not math.isfinite(baseline_length_m) or baseline_length_m <= 1.0e-12:
            return None, "zero_raw_gps_baseline", provenance
        left_center = self.node.nominal_left_camera.center_world_enu_m
        right_center = tuple(left_center[index] + baseline_enu[index] for index in range(3))
        poses = (
            replace(self.node.desired_left_camera, center_world_enu_m=left_center),
            replace(self.node.desired_right_camera, center_world_enu_m=right_center),
        )
        nominal_baseline = tuple(
            self.node.nominal_right_camera.center_world_enu_m[index]
            - self.node.nominal_left_camera.center_world_enu_m[index]
            for index in range(3)
        )
        baseline_delta = tuple(baseline_enu[index] - nominal_baseline[index] for index in range(3))
        provenance.update(
            {
                "gps_baseline_source": source,
                "gps_baseline_world_enu_m": baseline_enu,
                "gps_baseline_length_m": baseline_length_m,
                "gps_baseline_delta_world_enu_m": baseline_delta,
                "gps_baseline_delta_norm_m": norm_m(baseline_delta),
                "gps_left_eph_m": left_fix.eph_m,
                "gps_right_eph_m": right_fix.eph_m,
            }
        )
        return poses, "available", provenance

    def _uam_raw_gps_reference(self):
        """Project the UAM raw-GPS point into the fixed public FRD frame.

        The left-camera raw GPS is the local ENU origin.  Its nominal world-ENU
        optical center anchors that local tangent plane to Simulation's frozen
        ``baseline_center_frd`` output contract.  This is a GPS reference, not
        simulator ground truth.
        """

        if not getattr(self.node, "enable_gps_baseline_triangulation", False):
            return {"computation_status": "disabled"}
        with self.node.lock:
            left_entry = self.node.gps_baseline_fixes.get("left")
            uam_entry = self.node.gps_baseline_fixes.get("uam")
        if left_entry is None or uam_entry is None:
            return {"computation_status": "unavailable_raw_gps_fix"}
        current_epoch = getattr(getattr(self.node, "simulation_epoch", None), "epoch", 0)
        left_fix, left_received_ns, left_epoch, _ = self.node._gps_fix_entry(
            left_entry, current_epoch
        )
        uam_fix, uam_received_ns, uam_epoch, _ = self.node._gps_fix_entry(
            uam_entry, current_epoch
        )
        if left_epoch != current_epoch or uam_epoch != current_epoch:
            return {"computation_status": "raw_gps_epoch_mismatch"}
        now_ns = time.monotonic_ns()
        left_age_ms = max(0.0, (now_ns - left_received_ns) / 1_000_000.0)
        uam_age_ms = max(0.0, (now_ns - uam_received_ns) / 1_000_000.0)
        pair_skew_ms = abs(left_fix.timestamp_us - uam_fix.timestamp_us) / 1000.0
        if max(left_age_ms, uam_age_ms) > self.node.gps_baseline_max_age_s * 1000.0:
            return {"computation_status": "stale_raw_gps_fix"}
        if (
            abs(left_fix.timestamp_us - uam_fix.timestamp_us)
            > self.node.gps_baseline_max_age_s * 1_000_000
        ):
            return {"computation_status": "unsynchronized_raw_gps_fix"}
        try:
            uam_from_left_enu = gps_relative_enu_m(left_fix, uam_fix)
        except ValueError:
            return {"computation_status": "invalid_raw_gps_fix"}
        uam_world_enu = tuple(
            self.node.nominal_left_camera.center_world_enu_m[index] + uam_from_left_enu[index]
            for index in range(3)
        )
        forward_m, right_m, down_m = self.node.baseline_center_frd.point_from_world(
            uam_world_enu
        )
        return {
            "computation_status": "computed",
            "forward_m": forward_m,
            "right_m": right_m,
            "down_m": down_m,
            "range_m": norm_m((forward_m, right_m, down_m)),
            "world_enu_m": uam_world_enu,
            "left_timestamp_us": left_fix.timestamp_us,
            "uam_timestamp_us": uam_fix.timestamp_us,
            "left_eph_m": left_fix.eph_m,
            "uam_eph_m": uam_fix.eph_m,
            "left_age_ms": left_age_ms,
            "uam_age_ms": uam_age_ms,
            "pair_skew_ms": pair_skew_ms,
        }
