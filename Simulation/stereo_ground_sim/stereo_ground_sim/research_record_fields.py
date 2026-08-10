"""Pure schema-field builders for research recording."""

import math

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME
from stereo_ground_sim.coordinate_frames import VEHICLE_REFERENCE_LINK


def diagnostic_ray_record_fields(name, result):
    """Flatten one baseline-center FRD diagnostic ray into stable columns."""

    result = {} if result is None else result
    fields = {
        f"{name}_valid": bool(result.get("valid", False)),
        f"{name}_computation_status": result.get("computation_status", "invalid_input"),
        f"{name}_failure_reason": result.get("failure_reason", "invalid_input"),
    }
    for field in (
        "forward_m", "right_m", "down_m", "range_m", "ray_gap_m",
        "intersection_angle_deg", "left_depth_m", "right_depth_m",
        "geometry_rank", "condition_number",
    ):
        fields[f"{name}_{field}"] = result.get(field)
    for source, target in (
        ("midpoint_baseline_center_frd_m", "midpoint"),
        ("point_on_left_ray_baseline_center_frd_m", "left_closest"),
        ("point_on_right_ray_baseline_center_frd_m", "right_closest"),
    ):
        values = result.get(source)
        for index, axis in enumerate(("x", "y", "z")):
            fields[f"{name}_{target}_{axis}_m"] = (
                values[index]
                if isinstance(values, (tuple, list)) and len(values) >= 3
                else None
            )
    provenance = (
        "pixel_source", "attitude_source", "baseline_source",
        "baseline_anchor_policy", "left_camera_center_source",
        "right_camera_center_source", "left_attitude_timestamp_us",
        "right_attitude_timestamp_us", "left_attitude_simulation_timestamp_ns",
        "right_attitude_simulation_timestamp_ns", "left_attitude_receive_monotonic_ns",
        "right_attitude_receive_monotonic_ns", "left_attitude_reset_counter",
        "right_attitude_reset_counter", "left_attitude_age_ms",
        "right_attitude_age_ms", "attitude_pair_skew_ms", "left_gps_age_ms",
        "right_gps_age_ms", "gps_pair_skew_ms", "gps_left_timestamp_us",
        "gps_right_timestamp_us", "simulation_epoch", "px4_quaternion_order",
        "px4_attitude_semantics",
    )
    for field in provenance:
        fields[f"{name}_{field}"] = result.get(field)
    for label in ("left", "right"):
        for axis in ("w", "x", "y", "z"):
            fields[f"{name}_{label}_px4_q_{axis}"] = result.get(
                f"{label}_px4_q_{axis}"
            )
    return fields

def calibration_research_record_fields(pair, alignment, metrics):
    """Flatten the reality-transferable Operational input contract.

    Values here intentionally come only from observed pixels, PX4 attitude,
    and the GPS-baseline operational pose. Gazebo optical pose and injected
    truth remain separate label/debug fields.
    """

    rays = metrics.get("_diagnostic_rays")
    operational = rays.get("operational", {}) if isinstance(rays, dict) else {}
    status = str(operational.get("computation_status") or "unavailable")
    op_valid = bool(operational.get("valid", False))
    gt_mode = "unavailable" if alignment is None else alignment.gt_alignment_mode
    gt_before = None if alignment is None else alignment.gt_before_stamp_ns
    gt_after = None if alignment is None else alignment.gt_after_stamp_ns
    gt_timestamp = (
        pair.pair_midpoint_stamp_ns if gt_mode in ("exact", "interpolated")
        else gt_before if gt_mode == "nearest_before"
        else gt_after if gt_mode == "nearest_after"
        else None
    )
    gt_world = None if alignment is None else alignment.position_world_enu_m
    left_pose_ns = operational.get("left_attitude_simulation_timestamp_ns")
    right_pose_ns = operational.get("right_attitude_simulation_timestamp_ns")
    left_position_ns = operational.get("left_gps_simulation_timestamp_ns")
    right_position_ns = operational.get("right_gps_simulation_timestamp_ns")
    source_snapshot_ns = operational.get(
        "operational_source_snapshot_timestamp_ns"
    )
    left_depth = operational.get("left_depth_m")
    right_depth = operational.get("right_depth_m")
    observation_valid = bool(
        metrics.get("left_observation_valid")
        and metrics.get("right_observation_valid")
    )
    if observation_valid:
        observation_failure_reason = ""
    elif not metrics.get("left_observation_valid") and not metrics.get("right_observation_valid"):
        observation_failure_reason = "missing_left_and_right_observation"
    elif not metrics.get("left_observation_valid"):
        observation_failure_reason = "missing_left_observation"
    else:
        observation_failure_reason = "missing_right_observation"
    failure_reason = str(
        operational.get("failure_reason")
        or ("" if op_valid else observation_failure_reason or status)
    )
    failure_token = f"{status};{failure_reason}".lower()
    fields = {
        "sample_id": f"{pair.epoch}:{pair.pair_id}",
        "simulation_timestamp_ns": int(pair.pair_midpoint_stamp_ns),
        "left_observation_timestamp_ns": int(pair.left_stamp_ns),
        "right_observation_timestamp_ns": int(pair.right_stamp_ns),
        "left_right_dt_s": float(pair.pair_delta_ns) / 1_000_000_000.0,
        "left_pose_alignment_dt_s": (
            None if left_pose_ns is None else (int(left_pose_ns) - pair.pair_midpoint_stamp_ns) / 1_000_000_000.0
        ),
        "right_pose_alignment_dt_s": (
            None if right_pose_ns is None else (int(right_pose_ns) - pair.pair_midpoint_stamp_ns) / 1_000_000_000.0
        ),
        "left_position_alignment_dt_s": (
            None if left_position_ns is None else (int(left_position_ns) - pair.pair_midpoint_stamp_ns) / 1_000_000_000.0
        ),
        "right_position_alignment_dt_s": (
            None if right_position_ns is None else (int(right_position_ns) - pair.pair_midpoint_stamp_ns) / 1_000_000_000.0
        ),
        "observation_gt_dt_s": (
            None if gt_timestamp is None else (int(gt_timestamp) - pair.pair_midpoint_stamp_ns) / 1_000_000_000.0
        ),
        "sync_valid": (
            bool(pair.pairing_mode)
            and gt_mode != "unavailable"
            and op_valid
            and left_pose_ns is not None
            and right_pose_ns is not None
            and left_position_ns is not None
            and right_position_ns is not None
        ),
        "sync_failure_reason": (
            "" if gt_mode != "unavailable" and op_valid
            and left_pose_ns is not None and right_pose_ns is not None
            and left_position_ns is not None and right_position_ns is not None
            else "ground_truth_alignment_unavailable" if gt_mode == "unavailable"
            else str(operational.get("failure_reason") or status) if not op_valid
            else "operational_pose_timestamp_unavailable"
        ),
        "interpolation_method": gt_mode,
        "source_timestamp_before_ns": gt_before,
        "source_timestamp_after_ns": gt_after,
        "interpolation_alpha": None if alignment is None else alignment.gt_interpolation_alpha,
        "left_observation_source": metrics.get("left_observation_source"),
        "right_observation_source": metrics.get("right_observation_source"),
        "left_observation_valid": metrics.get("left_observation_valid"),
        "right_observation_valid": metrics.get("right_observation_valid"),
        "left_bbox_xmin_px": metrics.get("left_bbox_xmin_px"),
        "left_bbox_ymin_px": metrics.get("left_bbox_ymin_px"),
        "left_bbox_xmax_px": metrics.get("left_bbox_xmax_px"),
        "left_bbox_ymax_px": metrics.get("left_bbox_ymax_px"),
        "right_bbox_xmin_px": metrics.get("right_bbox_xmin_px"),
        "right_bbox_ymin_px": metrics.get("right_bbox_ymin_px"),
        "right_bbox_xmax_px": metrics.get("right_bbox_xmax_px"),
        "right_bbox_ymax_px": metrics.get("right_bbox_ymax_px"),
        "operational_position_source": operational.get("baseline_source"),
        "operational_position_frame": BASELINE_CENTER_FRD_FRAME,
        "operational_source_snapshot_timestamp_ns": source_snapshot_ns,
        "left_operational_position_timestamp_ns": left_position_ns,
        "right_operational_position_timestamp_ns": right_position_ns,
        "left_operational_position_valid": op_valid,
        "right_operational_position_valid": op_valid,
        "operational_attitude_source": operational.get("attitude_source"),
        "operational_attitude_parent_frame": "earth_ned",
        "operational_attitude_child_frame": "body_frd",
        "operational_attitude_quaternion_order": operational.get("px4_quaternion_order"),
        "left_operational_pose_timestamp_ns": left_pose_ns,
        "right_operational_pose_timestamp_ns": right_pose_ns,
        "left_operational_attitude_timestamp_ns": left_pose_ns,
        "right_operational_attitude_timestamp_ns": right_pose_ns,
        "left_operational_pose_valid": op_valid,
        "right_operational_pose_valid": op_valid,
        "operational_x_m": operational.get("forward_m"),
        "operational_y_m": operational.get("right_m"),
        "operational_z_m": operational.get("down_m"),
        "operational_range_m": operational.get("range_m"),
        "operational_azimuth_deg": None,
        "operational_elevation_deg": None,
        "operational_ray_generation_valid": op_valid,
        "triangulation_valid": op_valid,
        "triangulation_status": status,
        "triangulation_failure_reason": failure_reason,
        "near_parallel": "parallel" in failure_token,
        "negative_left_depth": None if left_depth is None else float(left_depth) < 0.0,
        "negative_right_depth": None if right_depth is None else float(right_depth) < 0.0,
        "nonfinite_input": "nonfinite_input" in failure_token or "invalid_input" in failure_token,
        "nonfinite_output": "nonfinite_output" in failure_token,
        "zero_or_small_baseline": "baseline" in failure_token,
        "condition_exceeded": "condition" in failure_token,
        "sample_valid": observation_valid and op_valid and gt_mode != "unavailable",
        "failure_stage": (
            "" if observation_valid and op_valid
            else "observation" if not observation_valid
            else "operational_triangulation"
        ),
        "failure_code": (
            "" if observation_valid and op_valid
            else observation_failure_reason if not observation_valid
            else status
        ),
        "failure_reason": "" if op_valid else failure_reason,
        "gt_timestamp_ns": gt_timestamp,
        "gt_valid": gt_world is not None,
        "gt_model_name": metrics.get("target_model_name"),
        "gt_link_name": VEHICLE_REFERENCE_LINK,
        "gt_world_enu_x_m": None if gt_world is None else gt_world[0],
        "gt_world_enu_y_m": None if gt_world is None else gt_world[1],
        "gt_world_enu_z_m": None if gt_world is None else gt_world[2],
        "left_injected_roll_deg": metrics.get("cam_l_optical_roll_deg"),
        "left_injected_pitch_deg": metrics.get("cam_l_optical_pitch_deg"),
        "left_injected_yaw_deg": metrics.get("cam_l_optical_yaw_deg"),
        "right_injected_roll_deg": metrics.get("cam_r_optical_roll_deg"),
        "right_injected_pitch_deg": metrics.get("cam_r_optical_pitch_deg"),
        "right_injected_yaw_deg": metrics.get("cam_r_optical_yaw_deg"),
    }
    for side in ("left", "right"):
        origin = operational.get(f"{side}_ray_origin_baseline_center_frd_m")
        for axis, value in zip(("x", "y", "z"), origin or ()):
            fields[f"{side}_operational_position_baseline_center_frd_{axis}_m"] = value
        for axis in ("x", "y", "z"):
            fields.setdefault(f"{side}_operational_position_baseline_center_frd_{axis}_m", None)
        for axis in ("w", "x", "y", "z"):
            fields[f"{side}_operational_attitude_q{axis}"] = operational.get(
                f"{side}_px4_q_{axis}"
            )
    forward, right, down = (operational.get(name) for name in ("forward_m", "right_m", "down_m"))
    if all(value is not None and math.isfinite(float(value)) for value in (forward, right, down)):
        fields["operational_azimuth_deg"] = math.degrees(math.atan2(float(right), float(forward)))
        fields["operational_elevation_deg"] = math.degrees(math.atan2(-float(down), math.hypot(float(forward), float(right))))
    return fields



def camera_oracle_record_fields(snapshot, *, expected_sdf_sha256=""):
    """Flatten world-ENU camera positions and XYZW orientations."""

    fields = {
        "camera_oracle_source": "unavailable",
        "camera_oracle_expected_sdf_sha256": str(expected_sdf_sha256),
    }
    for label in ("left", "right"):
        for name in (
            "gazebo_stamp_ns", "base_entity_name", "base_entity_id",
            "optical_entity_name", "optical_entity_id", "optical_x_m",
            "optical_y_m", "optical_z_m", "optical_qx", "optical_qy",
            "optical_qz", "optical_qw",
        ):
            fields[f"{label}_camera_oracle_{name}"] = None
    if snapshot is None:
        return fields
    fields["camera_oracle_source"] = "gazebo_observed_optical"
    for label, observation in zip(("left", "right"), snapshot):
        position = observation.optical_position_world_enu_m
        orientation = observation.optical_orientation_world_from_entity_xyzw
        values = {
            "gazebo_stamp_ns": observation.gazebo_stamp_ns,
            "base_entity_name": observation.base_entity_name,
            "base_entity_id": observation.base_entity_id,
            "optical_entity_name": observation.optical_entity_name,
            "optical_entity_id": observation.optical_entity_id,
            "optical_x_m": position[0], "optical_y_m": position[1],
            "optical_z_m": position[2], "optical_qx": orientation[0],
            "optical_qy": orientation[1], "optical_qz": orientation[2],
            "optical_qw": orientation[3],
        }
        fields.update(
            {f"{label}_camera_oracle_{name}": value for name, value in values.items()}
        )
    return fields


def diagnostic_research_record_fields(metrics):
    """Flatten UAM world-ENU reference and diagnostic FRD ray records."""

    uam_names = (
        "computation_status", "forward_m", "right_m", "down_m", "range_m",
        "left_timestamp_us", "timestamp_us", "left_age_ms", "uam_age_ms",
        "pair_skew_ms", "left_eph_m", "uam_eph_m",
    )
    fields = {
        f"uam_raw_gps_reference_{name}": metrics.get(
            f"uam_raw_gps_reference_{name}"
        )
        for name in uam_names
    }
    world_enu = metrics.get("uam_raw_gps_reference_world_enu_m")
    for index, axis in enumerate(("x", "y", "z")):
        fields[f"uam_raw_gps_reference_world_enu_{axis}_m"] = (
            world_enu[index]
            if isinstance(world_enu, (tuple, list)) and len(world_enu) >= 3
            else None
        )
    rays = metrics.get("_diagnostic_rays")
    if not isinstance(rays, dict):
        return fields
    for name in ("attitude", "operational"):
        result = rays.get(name)
        if not isinstance(result, dict):
            continue
        prefix = f"{name}_ray"
        fields.update(
            {
                f"{prefix}_valid": bool(result.get("valid", False)),
                f"{prefix}_computation_status": result.get("computation_status"),
                f"{prefix}_failure_reason": result.get("failure_reason"),
                f"{prefix}_source": ";".join(
                    str(result.get(key, ""))
                    for key in ("pixel_source", "attitude_source", "baseline_source")
                ),
                f"{prefix}_baseline_source": result.get("baseline_source"),
                f"{prefix}_gps_baseline_source": result.get("gps_baseline_source"),
                f"{prefix}_pixel_source": result.get("pixel_source"),
                f"{prefix}_attitude_source": result.get("attitude_source"),
                f"{prefix}_camera_center_source": ";".join(
                    str(result.get(key, ""))
                    for key in ("left_camera_center_source", "right_camera_center_source")
                ),
            }
        )
        for side in ("left", "right"):
            for source, target, unit in (
                (f"{side}_ray_origin_baseline_center_frd_m", "origin", "_m"),
                (f"{side}_ray_direction_baseline_center_frd", "direction", ""),
            ):
                vector = result.get(source)
                for index, axis in enumerate(("x", "y", "z")):
                    fields[f"{prefix}_{side}_{target}_{axis}{unit}"] = (
                        vector[index]
                        if isinstance(vector, (tuple, list)) and len(vector) >= 3
                        else None
                    )
        for field in (
            "forward_m", "right_m", "down_m", "range_m",
            "intersection_angle_deg", "left_depth_m", "right_depth_m",
            "geometry_rank", "condition_number",
        ):
            fields[f"{prefix}_{field}"] = result.get(field)
        fields[f"{prefix}_gap_m"] = result.get("ray_gap_m")
        for side in ("left", "right"):
            fields[f"{prefix}_{side}_attitude_truth_residual_deg"] = result.get(
                f"{side}_attitude_truth_residual_deg"
            )
        for source, target in (
            ("midpoint_baseline_center_frd_m", "midpoint"),
            ("point_on_left_ray_baseline_center_frd_m", "left_closest"),
            ("point_on_right_ray_baseline_center_frd_m", "right_closest"),
        ):
            vector = result.get(source)
            for index, axis in enumerate(("x", "y", "z")):
                fields[f"{prefix}_{target}_{axis}_m"] = (
                    vector[index]
                    if isinstance(vector, (tuple, list)) and len(vector) >= 3
                    else None
                )
        fields[f"{prefix}_delta_valid"] = metrics.get(f"{name}_delta_valid")
        fields[f"{prefix}_delta_failure_reason"] = metrics.get(
            f"{name}_delta_failure_reason"
        )
        for field in (
            "delta_forward_m", "delta_right_m", "delta_down_m",
            "delta_range_m", "position_difference_m", "gt_forward_error_m",
            "gt_right_error_m", "gt_down_error_m", "gt_range_error_m",
            "gt_position_error_m",
        ):
            fields[f"{prefix}_{field}"] = metrics.get(f"{name}_{field}")
        fields[f"{prefix}_gt_valid"] = metrics.get(f"{name}_gt_valid")
        fields[f"{prefix}_gt_failure_reason"] = metrics.get(f"{name}_gt_failure_reason")
        for source in ("gps_baseline_world_enu_m", "gps_baseline_delta_world_enu_m"):
            vector = result.get(source)
            target = source.removesuffix("_m")
            for index, axis in enumerate(("x", "y", "z")):
                fields[f"{prefix}_{target}_{axis}_m"] = (
                    vector[index]
                    if isinstance(vector, (tuple, list)) and len(vector) >= 3
                    else None
                )
        fields[f"{prefix}_gps_baseline_length_m"] = result.get("gps_baseline_length_m")
        fields[f"{prefix}_gps_baseline_delta_norm_m"] = result.get("gps_baseline_delta_norm_m")

    attitude_candidates = tuple(
        item for item in (rays.get("attitude", {}), rays.get("operational", {}))
        if isinstance(item, dict)
    )
    gps_candidates = tuple(
        item for item in (rays.get("gps_baseline", {}), rays.get("operational", {}))
        if isinstance(item, dict)
    )
    attitude = next(
        (item for item in attitude_candidates if item.get("left_attitude_timestamp_us") is not None or item.get("right_attitude_timestamp_us") is not None),
        attitude_candidates[0] if attitude_candidates else {},
    )
    gps = next(
        (item for item in gps_candidates if item.get("gps_left_timestamp_us") is not None or item.get("gps_right_timestamp_us") is not None),
        gps_candidates[0] if gps_candidates else {},
    )
    for label in ("left", "right"):
        for axis in ("w", "x", "y", "z"):
            fields[f"{label}_px4_q_{axis}"] = attitude.get(f"{label}_px4_q_{axis}")
        for suffix in (
            "attitude_timestamp_us", "attitude_receive_monotonic_ns",
            "attitude_reset_counter", "attitude_age_ms",
        ):
            fields[f"{label}_{suffix}"] = attitude.get(f"{label}_{suffix}")
        fields[f"{label}_gps_timestamp_us"] = gps.get(f"gps_{label}_timestamp_us")
        fields[f"{label}_gps_age_ms"] = gps.get(f"{label}_gps_age_ms")
    fields.update(
        {
            "px4_quaternion_order": attitude.get("px4_quaternion_order"),
            "px4_attitude_semantics": attitude.get("px4_attitude_semantics"),
            "attitude_pair_skew_ms": attitude.get("attitude_pair_skew_ms"),
            "gps_pair_skew_ms": gps.get("gps_pair_skew_ms"),
            "simulation_epoch": metrics.get("epoch"),
            "baseline_anchor_policy": gps.get("baseline_anchor_policy"),
        }
    )
    return fields
