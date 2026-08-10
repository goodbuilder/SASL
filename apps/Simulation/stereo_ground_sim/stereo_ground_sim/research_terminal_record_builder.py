"""Build one immutable terminal research record outside the ROS node."""

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME
from stereo_ground_sim.coordinate_frames import (
    SOURCE_GAZEBO_ENTITY,
    VEHICLE_REFERENCE_DEFINITION,
    VEHICLE_REFERENCE_LINK,
)
from stereo_ground_sim.research_recording import (
    RESEARCH_SCHEMA_VERSION,
    empty_pair_record,
)
from stereo_ground_sim.research_record_policy import sample_eligible


PX4_ATTITUDE_MODEL_VERSION = "configured_independent_body_attitude_v2"
PX4_ATTITUDE_REFERENCE_SOURCE = "configured_px4_attitude_rpy"
PX4_ATTITUDE_FRAME = "body_frd"
PX4_ATTITUDE_COMPOSITION = "canonical_rig_heading_plus_configured_rpy"
PX4_ATTITUDE_TO_LEGACY_RESEARCH_COLUMN = {
    "cam_l_px4_attitude_roll_deg": "cam_l_px4_roll_deg",
    "cam_l_px4_attitude_pitch_deg": "cam_l_px4_pitch_deg",
    "cam_l_px4_attitude_yaw_deg": "cam_l_px4_yaw_deg",
    "cam_r_px4_attitude_roll_deg": "cam_r_px4_roll_deg",
    "cam_r_px4_attitude_pitch_deg": "cam_r_px4_pitch_deg",
    "cam_r_px4_attitude_yaw_deg": "cam_r_px4_yaw_deg",
}


def build_terminal_record(
    node, pair, *, terminal_status, terminal_reason, measurement_status,
    evaluation_status, metrics=None, alignment=None, recorder,
    camera_oracle_unset,
):
    metrics = {} if metrics is None else dict(metrics)
    observation = node._observation_metadata()
    alignment = alignment or getattr(node, "gt_alignments_by_pair_id", {}).get(
        pair.pair_id
    )
    run_id = str(getattr(node, "research_run_id", "") or "legacy-test-run")
    case_id = str(getattr(node, "research_case_id", "") or "legacy-test-case")
    record = empty_pair_record()
    record.update(
        {
            "schema_version": RESEARCH_SCHEMA_VERSION,
            "session_id": str(
                getattr(node, "research_session_id", "") or run_id
            ),
            "run_id": run_id,
            "attempt_id": str(
                getattr(node, "research_attempt_id", "") or case_id
            ),
            "case_id": case_id,
            "effective_config_sha256": str(
                getattr(node, "research_effective_config_sha256", "")
                or ("0" * 64)
            ),
            "record_sequence": None,
            "record_type": "pair_terminal",
            "pair_id": int(pair.pair_id),
            "epoch": int(pair.epoch),
            "left_stamp_ns": int(pair.left_stamp_ns),
            "right_stamp_ns": int(pair.right_stamp_ns),
            "pair_midpoint_stamp_ns": int(pair.pair_midpoint_stamp_ns),
            "pair_delta_ns": int(pair.pair_delta_ns),
            "pairing_mode": str(pair.pairing_mode),
            "terminal_status": str(terminal_status),
            "terminal_reason": str(terminal_reason),
            "mission_phase": metrics.get("mission_phase", node._recording_mission_phase()),
            "measurement_status": str(measurement_status),
            "evaluation_status": str(evaluation_status),
            "gt_alignment_mode": (
                "unavailable" if alignment is None else alignment.gt_alignment_mode
            ),
            "gt_before_stamp_ns": None if alignment is None else alignment.gt_before_stamp_ns,
            "gt_after_stamp_ns": None if alignment is None else alignment.gt_after_stamp_ns,
            "gt_interpolation_alpha": None if alignment is None else alignment.gt_interpolation_alpha,
            "gt_nearest_delta_ns": None if alignment is None else alignment.gt_nearest_delta_ns,
            "observation_mode": metrics.get(
                "observation_mode", observation["observation_mode"]
            ),
            "observation_source": metrics.get(
                "observation_source", observation["observation_source"]
            ),
            "measurement_gt_independent": metrics.get(
                "measurement_gt_independent",
                observation["measurement_gt_independent"],
            ),
            "reference_source": metrics.get(
                "reference_source", observation["reference_source"]
            ),
            "projection_camera_pose_source": metrics.get(
                "projection_camera_pose_source",
                observation["projection_camera_pose_source"],
            ),
            "left_u_px": metrics.get("left_u_px"),
            "left_v_px": metrics.get("left_v_px"),
            "right_u_px": metrics.get("right_u_px"),
            "right_v_px": metrics.get("right_v_px"),
            "left_confidence": metrics.get("left_confidence"),
            "right_confidence": metrics.get("right_confidence"),
            "nominal_pose_computation_status": metrics.get(
                "raw_computation_status", metrics.get("computation_status")
            ),
            "nominal_pose_forward_m": metrics.get("raw_forward_m", metrics.get("estimated_forward_m")),
            "nominal_pose_right_m": metrics.get("raw_right_m", metrics.get("estimated_right_m")),
            "nominal_pose_down_m": metrics.get("raw_down_m", metrics.get("estimated_down_m")),
            "nominal_pose_range_m": metrics.get("raw_range_m", metrics.get("estimated_range_m")),
            "nominal_pose_ray_gap_m": metrics.get("raw_ray_gap_m", metrics.get("ray_gap_m")),
            "nominal_pose_intersection_angle_deg": metrics.get(
                "raw_intersection_angle_deg", metrics.get("intersection_angle_deg")
            ),
            "nominal_pose_left_depth_m": metrics.get("raw_left_depth_m", metrics.get("left_depth_m")),
            "nominal_pose_right_depth_m": metrics.get("raw_right_depth_m", metrics.get("right_depth_m")),
            "nominal_pose_geometry_rank": metrics.get("raw_geometry_rank", metrics.get("geometry_rank")),
            "nominal_pose_condition_number": metrics.get(
                "raw_condition_number", metrics.get("condition_number")
            ),
            "corrected_computation_status": metrics.get(
                "attitude_corrected_computation_status"
            ),
            "corrected_forward_m": metrics.get("attitude_corrected_forward_m"),
            "corrected_right_m": metrics.get("attitude_corrected_right_m"),
            "corrected_down_m": metrics.get("attitude_corrected_down_m"),
            "corrected_range_m": metrics.get("attitude_corrected_range_m"),
            "corrected_ray_gap_m": metrics.get("attitude_corrected_ray_gap_m"),
            "corrected_intersection_angle_deg": metrics.get(
                "attitude_corrected_intersection_angle_deg"
            ),
            "corrected_left_depth_m": metrics.get("attitude_corrected_left_depth_m"),
            "corrected_right_depth_m": metrics.get("attitude_corrected_right_depth_m"),
            "corrected_geometry_rank": metrics.get("attitude_corrected_geometry_rank"),
            "corrected_condition_number": metrics.get(
                "attitude_corrected_condition_number"
            ),
            "gt_source": metrics.get(
                "gt_source", "gazebo_vehicle_reference_link_world_pose"
            ),
            "gt_source_entity": metrics.get(
                "gt_source_entity", SOURCE_GAZEBO_ENTITY
            ),
            "gt_reference_name": metrics.get(
                "gt_reference_name", VEHICLE_REFERENCE_LINK
            ),
            "gt_reference_definition": metrics.get(
                "gt_reference_definition", VEHICLE_REFERENCE_DEFINITION
            ),
            "gt_fallback_used": bool(metrics.get("gt_fallback_used", False)),
            "gt_forward_m": metrics.get("gt_forward_m"),
            "gt_right_m": metrics.get("gt_right_m"),
            "gt_down_m": metrics.get("gt_down_m"),
            "gt_range_m": metrics.get("gt_range_m"),
            "nominal_pose_position_error_m": metrics.get("raw_position_error_m"),
            "nominal_pose_forward_error_m": metrics.get("raw_forward_error_m"),
            "nominal_pose_right_error_m": metrics.get("raw_right_error_m"),
            "nominal_pose_down_error_m": metrics.get("raw_down_error_m"),
            "nominal_pose_range_error_m": metrics.get("raw_range_error_m"),
            "corrected_position_error_m": metrics.get("corrected_position_error_m"),
            "corrected_forward_error_m": metrics.get("corrected_forward_error_m"),
            "corrected_right_error_m": metrics.get("corrected_right_error_m"),
            "corrected_down_error_m": metrics.get("corrected_down_error_m"),
            "corrected_range_error_m": metrics.get("corrected_range_error_m"),
            "measurement_queue_depth": len(getattr(node, "measurement_pair_queue", ())),
            "measurement_queue_depth_max": getattr(node, "measurement_queue_depth_max", 0),
            "measurement_queue_capacity": getattr(node, "measurement_queue_capacity", 0),
            "measurement_queue_overflow_count": getattr(
                node, "measurement_queue_overflow_count", 0
            ),
            "measurement_overrun_invalid": getattr(
                node, "measurement_overrun_invalid", False
            ),
            "measurement_rejected_after_overrun_count": getattr(
                node, "measurement_rejected_after_overrun_count", 0
            ),
            "measurement_accepted_count": getattr(node, "measurement_accepted_count", 0),
            "measurement_processed_count": getattr(node, "measurement_processed_count", 0),
            "accepted_pair_discard_count": getattr(
                node, "accepted_pair_discard_count", 0
            ),
            "measurement_processing_error_count": getattr(
                node, "measurement_processing_error_count", 0
            ),
            "gt_evaluation_success_count": getattr(
                node, "gt_evaluation_success_count", 0
            ),
            "gt_evaluation_error_count": getattr(node, "gt_evaluation_error_count", 0),
            "research_record_queue_depth": recorder.queue_depth,
            "research_record_queue_depth_max": recorder.queue_depth_max,
            "research_record_queue_capacity": recorder.queue_capacity,
            "research_recording_overrun_invalid": recorder.overrun_invalid,
            "research_recording_error_count": recorder.error_count,
            "output_frame": BASELINE_CENTER_FRD_FRAME,
            "output_axes": "forward_right_down",
            "position_unit": "m",
            "pixel_unit": "px",
            "angle_unit": "deg",
            "timestamp_unit": "ns",
            "correction_source": "gazebo_observed_optical_pose",
            "px4_attitude_model_version": PX4_ATTITUDE_MODEL_VERSION,
            "px4_attitude_reference_source": PX4_ATTITUDE_REFERENCE_SOURCE,
            "px4_attitude_frame": PX4_ATTITUDE_FRAME,
            "px4_attitude_composition": PX4_ATTITUDE_COMPOSITION,
            "px4_attitude_quaternion_order": "wxyz",
            "legacy_px4_absolute_attitude_used": True,
        }
    )
    record.update(
        node._camera_oracle_record_fields(
            metrics.get("camera_oracle_snapshot", camera_oracle_unset),
        )
    )
    record.update(node._diagnostic_research_record_fields(metrics))
    record.update(node._calibration_research_record_fields(pair, alignment, metrics))
    for name in node._attitude_parameter_names():
        record[
            PX4_ATTITUDE_TO_LEGACY_RESEARCH_COLUMN.get(name, name)
        ] = getattr(node, name)
    for name in (
        "effective_left_optical_roll_error_deg",
        "effective_left_optical_pitch_error_deg",
        "effective_left_optical_yaw_error_deg",
        "effective_right_optical_roll_error_deg",
        "effective_right_optical_pitch_error_deg",
        "effective_right_optical_yaw_error_deg",
    ):
        record[name] = getattr(node, name)

    record["sample_eligible"] = sample_eligible(
        record,
        terminal_status=terminal_status,
        measurement_status=measurement_status,
        evaluation_status=evaluation_status,
    )
    return record
