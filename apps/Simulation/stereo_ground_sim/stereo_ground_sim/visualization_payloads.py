"""Read-only serialization of FRD metrics for plot publishers."""

import math


def _number(value):
    return math.nan if value is None else float(value)


def _frd_range(metrics, prefix):
    """Serialize baseline-center `[forward,right,down,range]` metres."""

    return [
        _number(metrics.get(f"{prefix}{name}_m"))
        for name in ("forward", "right", "down", "range")
    ]


def plot_payloads(metrics, *, actual_fallback=None):
    """Build plot arrays without mutating measurement state."""

    actual = metrics.get("actual_distance_m", actual_fallback)
    distance = _number(metrics.get("distance_m"))
    error = _number(metrics.get("distance_error_m"))
    disparity = _number(metrics.get("disparity_px"))
    return {
        "distance": distance,
        "error": error,
        "disparity": disparity,
        "nominal_pose_frd_range": _frd_range(metrics, "estimated_"),
        "corrected_frd_range": _frd_range(metrics, "corrected_"),
        "optical_pose_frd_range": _frd_range(metrics, "optical_"),
        "ideal_frd_range": _frd_range(metrics, "ideal_"),
        "comparison": [_number(actual), distance, error, disparity],
        "bbox_adjustment": metrics.get("bbox_adjustment"),
    }


def json_number(value):
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def json_vector(values):
    if values is None:
        return None
    try:
        result = [json_number(value) for value in values[:3]]
    except (TypeError, ValueError):
        return None
    return result if len(result) == 3 and all(value is not None for value in result) else None


def ray_metrics_payload(result):
    """Serialize one baseline-center FRD ray result without changing it."""

    if result is None:
        return None
    payload = {
        "valid": bool(result.get("valid", result.get("computation_status") == "computed")),
        "computation_status": str(result.get("computation_status", "")),
        "failure_reason": str(result.get("failure_reason", "")),
        "delta_valid": bool(result.get("delta_valid", False)),
        "delta_failure_reason": str(result.get("delta_failure_reason", "")),
        "gt_valid": bool(result.get("gt_valid", False)),
        "gt_failure_reason": str(result.get("gt_failure_reason", "")),
        "forward_m": json_number(result.get("forward_m")),
        "right_m": json_number(result.get("right_m")),
        "down_m": json_number(result.get("down_m")),
        "range_m": json_number(result.get("range_m")),
        "gap_m": json_number(result.get("ray_gap_m")),
        "intersection_angle_deg": json_number(result.get("intersection_angle_deg")),
        "left_depth_m": json_number(result.get("left_depth_m")),
        "right_depth_m": json_number(result.get("right_depth_m")),
        "geometry_rank": json_number(result.get("geometry_rank")),
        "condition_number": json_number(result.get("condition_number")),
        "left_point_frd_m": json_vector(result.get("point_on_left_ray_baseline_center_frd_m")),
        "right_point_frd_m": json_vector(result.get("point_on_right_ray_baseline_center_frd_m")),
        "midpoint_frd_m": json_vector(result.get("midpoint_baseline_center_frd_m")),
    }
    numeric = (
        "delta_forward_m", "delta_right_m", "delta_down_m", "delta_range_m",
        "position_difference_m", "gt_forward_error_m", "gt_right_error_m",
        "gt_down_error_m", "gt_range_error_m", "gt_position_error_m",
        "left_attitude_age_ms", "right_attitude_age_ms", "attitude_pair_skew_ms",
        "left_gps_age_ms", "right_gps_age_ms", "gps_pair_skew_ms",
        "gps_baseline_length_m", "gps_baseline_delta_norm_m", "gps_left_eph_m",
        "gps_right_eph_m", "left_attitude_truth_residual_deg",
        "right_attitude_truth_residual_deg",
    )
    for name in numeric:
        if name in result:
            payload[name] = json_number(result.get(name))
    integers = (
        "left_attitude_timestamp_us", "right_attitude_timestamp_us",
        "left_attitude_receive_monotonic_ns", "right_attitude_receive_monotonic_ns",
        "left_attitude_reset_counter", "right_attitude_reset_counter",
        "gps_left_timestamp_us", "gps_right_timestamp_us", "simulation_epoch",
    )
    for name in integers:
        if result.get(name) is not None:
            payload[name] = int(result[name])
    for label in ("left", "right"):
        for axis in ("w", "x", "y", "z"):
            name = f"{label}_px4_q_{axis}"
            if name in result:
                payload[name] = json_number(result.get(name))
    strings = (
        "pixel_source", "attitude_source", "baseline_source",
        "baseline_anchor_policy", "left_camera_center_source",
        "right_camera_center_source", "px4_quaternion_order",
        "px4_attitude_semantics", "left_attitude_topic", "right_attitude_topic",
        "left_gps_topic", "right_gps_topic", "output_frame", "unit",
    )
    for name in strings:
        if name in result:
            payload[name] = str(result.get(name, ""))
    for name in ("gps_baseline_world_enu_m", "gps_baseline_delta_world_enu_m"):
        if name in result:
            payload[name] = json_vector(result.get(name))
    payload["provenance"] = ";".join(
        f"{name}={payload[name]}"
        for name in ("pixel_source", "attitude_source", "baseline_source")
        if name in payload
    )
    return payload


def ray_metrics_document(*, pair_id=None, gt_source=None, gt_fallback_used=None, **rays):
    payload = {}
    if pair_id is not None:
        payload["pair_id"] = int(pair_id)
    if gt_source is not None:
        payload["gt_source"] = str(gt_source)
    if gt_fallback_used is not None:
        payload["gt_fallback_used"] = bool(gt_fallback_used)
    for key, result in rays.items():
        ray_payload = ray_metrics_payload(result)
        if ray_payload is not None:
            payload[key] = ray_payload
            if key in ("attitude", "operational"):
                for field in (
                    "delta_forward_m", "delta_right_m", "delta_down_m",
                    "delta_range_m", "position_difference_m",
                ):
                    payload[f"{key}_{field}"] = ray_payload.get(field)
    ideal = rays.get("ideal")
    if ideal is not None:
        for source, target in (
            ("left_u_px", "ideal_left_u_px"), ("left_v_px", "ideal_left_v_px"),
            ("right_u_px", "ideal_right_u_px"), ("right_v_px", "ideal_right_v_px"),
        ):
            payload[target] = json_number(ideal.get(source))
        left_u, right_u = payload.get("ideal_left_u_px"), payload.get("ideal_right_u_px")
        payload["ideal_disparity_px"] = (
            None if left_u is None or right_u is None else left_u - right_u
        )
        payload["ideal_pixel_source"] = "ground_truth_world_enu_quantized_nominal_projection"
    return payload
