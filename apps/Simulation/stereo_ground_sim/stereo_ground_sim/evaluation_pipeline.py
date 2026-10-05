"""Pure error evaluation after Ground Truth has entered the output frame.

Both estimates and ``gt_baseline`` use baseline-center FRD axis order
``[forward, right, down]`` in metres. This module never accepts world ENU,
camera optical RDF, GPS, or PX4 NED vectors and performs no frame conversion.
"""

import math

from stereo_ground_sim.coordinate_frames import (
    SOURCE_GAZEBO_ENTITY,
    VEHICLE_REFERENCE_DEFINITION,
    VEHICLE_REFERENCE_LINK,
)


def _all_finite(values):
    return all(value is not None and math.isfinite(float(value)) for value in values)


def vector_error(metrics, prefix, gt_baseline):
    if gt_baseline is None:
        return None, None
    values = tuple(
        metrics.get(f"{prefix}{axis}_m") for axis in ("forward", "right", "down")
    )
    if not _all_finite(values):
        return None, None
    errors = tuple(
        float(values[index]) - float(gt_baseline[name])
        for index, name in enumerate(("forward_m", "right_m", "down_m"))
    )
    return math.sqrt(sum(value * value for value in errors)), errors[0]


def evaluation_metadata(pair, alignment, gt_baseline, metrics):
    def errors(prefix):
        if gt_baseline is None:
            return (None, None, None, None, None)
        estimate = tuple(
            metrics.get(f"{prefix}{axis}_m")
            for axis in ("forward", "right", "down")
        )
        if not _all_finite(estimate):
            return (None, None, None, None, None)
        component_errors = tuple(
            float(estimate[index]) - float(gt_baseline[name])
            for index, name in enumerate(("forward_m", "right_m", "down_m"))
        )
        estimated_range = metrics.get(f"{prefix}range_m")
        range_error = (
            float(estimated_range) - float(gt_baseline["range_m"])
            if _all_finite((estimated_range, gt_baseline["range_m"]))
            else None
        )
        return (
            math.sqrt(sum(value * value for value in component_errors)),
            *component_errors,
            range_error,
        )

    raw_errors = errors("estimated_")
    corrected_errors = errors("attitude_corrected_")
    ideal_errors = errors("ideal_")
    diagnostic_metadata = {}
    for name in ("attitude", "operational"):
        values = errors(f"{name}_")
        valid = values[0] is not None
        diagnostic_metadata.update(
            {
                f"{name}_gt_valid": valid,
                f"{name}_gt_failure_reason": "" if valid else "gt_or_ray_invalid",
                f"{name}_gt_position_error_m": values[0],
                f"{name}_gt_forward_error_m": values[1],
                f"{name}_gt_right_error_m": values[2],
                f"{name}_gt_down_error_m": values[3],
                f"{name}_gt_range_error_m": values[4],
            }
        )
    result = {
        "pair_id": pair.pair_id,
        "gt_alignment_mode": alignment.gt_alignment_mode,
        "gt_before_stamp_ns": alignment.gt_before_stamp_ns,
        "gt_after_stamp_ns": alignment.gt_after_stamp_ns,
        "gt_interpolation_alpha": alignment.gt_interpolation_alpha,
        "gt_nearest_delta_ns": alignment.gt_nearest_delta_ns,
        "gt_forward_m": None if gt_baseline is None else gt_baseline["forward_m"],
        "gt_right_m": None if gt_baseline is None else gt_baseline["right_m"],
        "gt_down_m": None if gt_baseline is None else gt_baseline["down_m"],
        "gt_range_m": None if gt_baseline is None else gt_baseline["range_m"],
        "gt_source": "gazebo_vehicle_reference_link_world_pose",
        "gt_source_entity": SOURCE_GAZEBO_ENTITY,
        "gt_reference_name": VEHICLE_REFERENCE_LINK,
        "gt_reference_definition": VEHICLE_REFERENCE_DEFINITION,
        "gt_fallback_used": False,
        **diagnostic_metadata,
    }
    for label, values in (
        ("raw", raw_errors),
        ("corrected", corrected_errors),
        ("ideal", ideal_errors),
    ):
        for suffix, value in zip(
            ("position", "forward", "right", "down", "range"), values
        ):
            result[f"{label}_{suffix}_error_m"] = value
    return result
