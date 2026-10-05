"""Observation-mode validation and immutable recording provenance.

Pixel coordinates use OpenCV image order ``(u=right, v=down)``.  This module
does not triangulate or transform coordinates; the declared 3-D output frame is
the baseline-center PX4/body-style FRD frame owned by ``sasl_common``.
"""

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME

from stereo_ground_sim.coordinate_frames import VEHICLE_REFERENCE_LINK


def validate_observation_mode(value):
    mode = str(value)
    if mode not in ("bbox_yolo", "ideal_reference_point"):
        raise ValueError("observation_mode must be bbox_yolo or ideal_reference_point")
    return mode


def validate_gps_baseline_source(value):
    source = str(value)
    if source not in ("gps_baseline", "global_baseline", "both"):
        raise ValueError(
            "gps_baseline_source must be gps_baseline, global_baseline, or both"
        )
    return source


def observation_metadata(mode):
    mode = validate_observation_mode(mode)
    ideal_mode = mode == "ideal_reference_point"
    return {
        "observation_mode": mode,
        "observation_source": (
            "gazebo_reference_point_projection" if ideal_mode else "yolo_bbox_center"
        ),
        "measurement_gt_independent": not ideal_mode,
        "reference_source": VEHICLE_REFERENCE_LINK if ideal_mode else "not_applicable",
        "projection_camera_pose_source": (
            "gazebo_observed_optical" if ideal_mode else "rendered_image_observation"
        ),
        "raw_interpretation_pose_source": "nominal_pose",
        "corrected_interpretation_pose_source": "gazebo_observed_optical",
        "ideal_reference_source": "ground_truth_world_enu_projected_nominal_pixels",
        "ideal_reference_runtime": "simulation_reference_only",
        "correction_source": "gazebo_observed_optical_pose",
        "evaluation_gt_source": "gazebo_reference_link_world_pose",
        "output_frame": BASELINE_CENTER_FRD_FRAME,
    }


def observation_record_fields(mode, left, right):
    """Build per-pair pixel/provenance fields without mutating detections."""

    mode = validate_observation_mode(mode)
    ideal_mode = mode == "ideal_reference_point"
    source = "gazebo_reference_point_projection" if ideal_mode else "yolo_bbox"
    fields = {
        "left_u_px": None if left is None else left.get("cx"),
        "left_v_px": None if left is None else left.get("cy"),
        "right_u_px": None if right is None else right.get("cx"),
        "right_v_px": None if right is None else right.get("cy"),
        "left_confidence": None if left is None else left.get("confidence"),
        "right_confidence": None if right is None else right.get("confidence"),
        "left_observation_source": source,
        "right_observation_source": source,
        "left_observation_valid": left is not None,
        "right_observation_valid": right is not None,
    }
    for label, detection in (("left", left), ("right", right)):
        bbox = None if detection is None else detection.get("bbox")
        if ideal_mode or not isinstance(bbox, (tuple, list)) or len(bbox) != 4:
            for suffix in ("xmin", "ymin", "xmax", "ymax"):
                fields[f"{label}_bbox_{suffix}_px"] = None
            continue
        for suffix, value in zip(("xmin", "ymin", "xmax", "ymax"), bbox):
            fields[f"{label}_bbox_{suffix}_px"] = float(value)
    return fields
