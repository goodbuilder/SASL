"""Pure validity and delta policy for baseline-center FRD diagnostic rays."""

import math

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME


class DiagnosticRayOrchestrator:
    """Compose diagnostic rays without owning pose or geometry state.

    Pose callbacks use world ENU camera centres with OpenCV optical RDF
    orientation. Triangulation results use baseline-center FRD metres. The GPS
    pose callback supplies an ENU separation anchored at the nominal left
    optical centre.
    """

    def __init__(self, dependencies, *, gps_baseline_anchor_policy):
        self.dependencies = dependencies
        self.gps_baseline_anchor_policy = gps_baseline_anchor_policy

    def compute(self, left_u, left_v, right_u, right_v, width, height, snapshot,
                *, operational_snapshot=None, observation_timestamp_ns=None):
        d = self.dependencies
        pixels = (left_u, left_v, right_u, right_v, width, height)
        attitude_poses, attitude_status, attitude_provenance = d["attitude_poses"](
            snapshot, include_provenance=True,
            operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
        )
        attitude_provenance.update({
            "baseline_source": "gazebo_observed_optical_centers",
            "left_camera_center_source": "gazebo_observed_left_optical_center",
            "right_camera_center_source": "gazebo_observed_right_optical_center",
        })
        if attitude_poses is None:
            attitude = finalize_diagnostic_ray(
                {"computation_status": attitude_status},
                provenance=attitude_provenance, failure_reason=attitude_status,
            )
        else:
            attitude = finalize_diagnostic_ray(
                d["triangulate"](
                    *pixels, left_camera_pose=attitude_poses[0],
                    right_camera_pose=attitude_poses[1],
                ),
                provenance=attitude_provenance,
            )

        true_north_poses, true_north_status, true_north_provenance = d["true_north_attitude_poses"](
            snapshot, include_provenance=True,
            operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
        )
        true_north_provenance.update({
            "baseline_source": "gazebo_observed_optical_centers",
            "left_camera_center_source": "gazebo_observed_left_optical_center",
            "right_camera_center_source": "gazebo_observed_right_optical_center",
        })
        if true_north_poses is None:
            attitude_true_north = finalize_diagnostic_ray(
                {"computation_status": true_north_status},
                provenance=true_north_provenance, failure_reason=true_north_status,
            )
        else:
            attitude_true_north = finalize_diagnostic_ray(
                d["triangulate"](
                    *pixels, left_camera_pose=true_north_poses[0],
                    right_camera_pose=true_north_poses[1],
                ),
                provenance=true_north_provenance,
            )

        declination_poses, declination_status, declination_provenance = d["declination_attitude_poses"](
            snapshot, include_provenance=True,
            operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
        )
        declination_provenance.update({
            "baseline_source": "gazebo_observed_optical_centers",
            "left_camera_center_source": "gazebo_observed_left_optical_center",
            "right_camera_center_source": "gazebo_observed_right_optical_center",
        })
        if declination_poses is None:
            attitude_declination = finalize_diagnostic_ray(
                {"computation_status": declination_status},
                provenance=declination_provenance, failure_reason=declination_status,
            )
        else:
            attitude_declination = finalize_diagnostic_ray(
                d["triangulate"](
                    *pixels, left_camera_pose=declination_poses[0],
                    right_camera_pose=declination_poses[1],
                ),
                provenance=declination_provenance,
            )

        gps_poses, gps_status, gps_provenance = d["gps_poses"](
            "gps_baseline", operational_snapshot=operational_snapshot,
            observation_timestamp_ns=observation_timestamp_ns,
        )
        full_poses, full_status, full_provenance = None, attitude_status, attitude_provenance
        if gps_poses is not None:
            full_poses, full_status, full_provenance = d["attitude_poses"](
                gps_poses, include_provenance=True,
                operational_snapshot=operational_snapshot,
                observation_timestamp_ns=observation_timestamp_ns,
            )
        provenance = {
            **gps_provenance, **full_provenance,
            "pixel_source": "observed",
            "attitude_source": "px4_vehicle_attitude",
            "baseline_source": "px4_raw_sensor_gps",
            "baseline_anchor_policy": self.gps_baseline_anchor_policy,
            "left_camera_center_source": "nominal_left_optical_center_anchor",
            "right_camera_center_source": "nominal_left_anchor_plus_px4_relative_enu_separation",
        }
        if gps_poses is None or full_poses is None:
            status = gps_status if gps_poses is None else full_status
            reasons = []
            if gps_poses is None:
                reasons.append(f"gps:{gps_status}")
            if full_poses is None:
                reasons.append(f"attitude:{full_status}")
            operational = finalize_diagnostic_ray(
                {"computation_status": status}, provenance=provenance,
                failure_reason=";".join(reasons),
            )
        else:
            operational = finalize_diagnostic_ray(
                d["triangulate"](
                    *pixels, left_camera_pose=full_poses[0],
                    right_camera_pose=full_poses[1],
                ),
                provenance=provenance,
            )
        true_north_full_poses, true_north_full_status, true_north_full_provenance = (
            None, true_north_status, true_north_provenance,
        )
        if gps_poses is not None:
            true_north_full_poses, true_north_full_status, true_north_full_provenance = d["true_north_attitude_poses"](
                gps_poses, include_provenance=True,
                operational_snapshot=operational_snapshot,
                observation_timestamp_ns=observation_timestamp_ns,
            )
        operational_true_north_provenance = {
            **gps_provenance, **true_north_full_provenance,
            "pixel_source": "observed",
            "baseline_source": "px4_raw_sensor_gps",
            "baseline_anchor_policy": self.gps_baseline_anchor_policy,
            "left_camera_center_source": "nominal_left_optical_center_anchor",
            "right_camera_center_source": "nominal_left_anchor_plus_px4_relative_enu_separation",
        }
        if gps_poses is None or true_north_full_poses is None:
            status = gps_status if gps_poses is None else true_north_full_status
            reasons = []
            if gps_poses is None:
                reasons.append(f"gps:{gps_status}")
            if true_north_full_poses is None:
                reasons.append(f"attitude:{true_north_full_status}")
            operational_true_north = finalize_diagnostic_ray(
                {"computation_status": status}, provenance=operational_true_north_provenance,
                failure_reason=";".join(reasons),
            )
        else:
            operational_true_north = finalize_diagnostic_ray(
                d["triangulate"](
                    *pixels, left_camera_pose=true_north_full_poses[0],
                    right_camera_pose=true_north_full_poses[1],
                ),
                provenance=operational_true_north_provenance,
            )

        return {
            "attitude": attitude,
            "operational": operational,
            "attitude_true_north": attitude_true_north,
            "attitude_declination": attitude_declination,
            "operational_true_north": operational_true_north,
        }


def _finite(values):
    return all(value is not None and math.isfinite(float(value)) for value in values)


def finalize_diagnostic_ray(result, *, provenance=None, failure_reason=""):
    finalized = {} if result is None else dict(result)
    if provenance:
        finalized.update(provenance)
    status = str(finalized.get("computation_status") or failure_reason or "invalid_input")
    frame_matches = finalized.get("output_frame") in (None, BASELINE_CENTER_FRD_FRAME)
    values = tuple(
        finalized.get(name) for name in ("forward_m", "right_m", "down_m", "range_m")
    )
    valid = status == "computed" and frame_matches and _finite(values)
    reason = "" if valid else (
        "frame_mismatch" if not frame_matches
        else str(failure_reason or finalized.get("diagnostic_error") or status)
    )
    finalized.update(
        {
            "valid": valid,
            "computation_status": status if frame_matches else "frame_mismatch",
            "failure_reason": reason,
            "output_frame": BASELINE_CENTER_FRD_FRAME,
            "unit": "m",
        }
    )
    return finalized


def diagnostic_delta(test, truth):
    empty = {
        "valid": False, "failure_reason": "",
        "delta_forward_m": None, "delta_right_m": None,
        "delta_down_m": None, "delta_range_m": None,
        "position_difference_m": None,
    }
    if not isinstance(truth, dict) or truth.get("computation_status") != "computed":
        return {**empty, "failure_reason": "truth_invalid"}
    if not isinstance(test, dict) or not bool(test.get("valid")):
        reason = "test_ray_invalid" if not isinstance(test, dict) else test.get("failure_reason")
        return {**empty, "failure_reason": str(reason or "test_ray_invalid")}
    if test.get("output_frame") != truth.get("output_frame"):
        return {**empty, "failure_reason": "frame_mismatch"}
    try:
        vector = tuple(
            float(test[key]) - float(truth[key])
            for key in ("forward_m", "right_m", "down_m")
        )
        delta_range = float(test["range_m"]) - float(truth["range_m"])
        if not _finite((*vector, delta_range)):
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return {**empty, "failure_reason": "invalid_delta_input"}
    return {
        "valid": True, "failure_reason": "",
        "delta_forward_m": vector[0], "delta_right_m": vector[1],
        "delta_down_m": vector[2], "delta_range_m": delta_range,
        "position_difference_m": math.sqrt(sum(value * value for value in vector)),
    }


def ray_gt_error(ray, gt_baseline):
    """Per-ray Ground Truth error: how far a ray's FRD estimate sits from GT.

    Mirrors ``diagnostic_delta`` but compares against the plain
    forward_m/right_m/down_m/range_m Ground Truth vector rather than another
    ray, so it applies uniformly to every recorded Ray (not just the two
    diagnostic-only rays that pipeline serves).

    Gated on ``computation_status == "computed"`` rather than a ``valid``
    key: only the diagnostic-only rays (attitude/operational, via
    ``finalize_diagnostic_ray``) carry a ``valid`` flag -- the plain
    triangulation output used by nominal_pose/optical_pose/ideal/gps_baseline
    never sets one, only ``computation_status``.
    """

    empty = {
        "gt_valid": False, "gt_failure_reason": "",
        "gt_forward_error_m": None, "gt_right_error_m": None,
        "gt_down_error_m": None, "gt_range_error_m": None,
        "gt_position_error_m": None,
    }
    if not isinstance(gt_baseline, dict):
        return {**empty, "gt_failure_reason": "gt_unavailable"}
    if not isinstance(ray, dict) or ray.get("computation_status") != "computed":
        reason = (
            "ray_invalid"
            if not isinstance(ray, dict)
            else str(ray.get("failure_reason") or ray.get("computation_status") or "ray_invalid")
        )
        return {**empty, "gt_failure_reason": reason}
    try:
        vector = tuple(
            float(ray[key]) - float(gt_baseline[key])
            for key in ("forward_m", "right_m", "down_m")
        )
        range_error = float(ray["range_m"]) - float(gt_baseline["range_m"])
        if not _finite((*vector, range_error)):
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return {**empty, "gt_failure_reason": "invalid_gt_input"}
    return {
        "gt_valid": True, "gt_failure_reason": "",
        "gt_forward_error_m": vector[0], "gt_right_error_m": vector[1],
        "gt_down_error_m": vector[2], "gt_range_error_m": range_error,
        "gt_position_error_m": math.sqrt(sum(value * value for value in vector)),
    }


def diagnostic_analysis(truth, diagnostic_rays):
    comparisons, flat = {}, {}
    for name in ("attitude", "operational"):
        comparison = diagnostic_delta(diagnostic_rays.get(name), truth)
        comparisons[name] = comparison
        flat[f"{name}_delta_valid"] = comparison["valid"]
        flat[f"{name}_delta_failure_reason"] = comparison["failure_reason"]
        for field in (
            "delta_forward_m", "delta_right_m", "delta_down_m",
            "delta_range_m", "position_difference_m",
        ):
            flat[f"{name}_{field}"] = comparison[field]
    return {"comparisons": comparisons, "flat": flat}
