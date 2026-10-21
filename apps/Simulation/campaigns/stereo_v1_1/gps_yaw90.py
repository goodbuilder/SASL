"""Measured-camera-baseline yaw for the offline Simulation campaign.

Positions are optical-center world ENU [east,north,up] metres. Headings
are clockwise from north, degrees; returned initial angles are radians.
The +/-90 branch scoring matches Workstation.workstation_core.baseline_yaw.
No camera/drone truth, image-derived position, or drone RTK is read here.
"""
from __future__ import annotations

import math


GPS_YAW90_CASE = "GPS_YAW90"


def select_yaw90(*, north_m, east_m, p1_yaw_deg=None, p2_yaw_deg=None,
                 reverse=False):
    """Return both baseline-perpendicular branches and frozen selection evidence."""
    north, east = float(north_m), float(east_m)
    if not all(math.isfinite(v) for v in (north, east)) or math.hypot(north, east) <= 1e-9:
        raise ValueError("camera baseline horizontal direction is undefined")
    azimuth = math.degrees(math.atan2(east, north)) % 360.0
    minus, plus = (azimuth - 90.0) % 360.0, (azimuth + 90.0) % 360.0
    references = []
    for value in (p1_yaw_deg, p2_yaw_deg):
        try:
            parsed = float(value)
        except (ValueError, TypeError):
            continue
        if math.isfinite(parsed):
            references.append(parsed % 360.0)
    def error(candidate, reference):
        return abs((candidate - reference + 180.0) % 360.0 - 180.0)
    minus_errors = [error(minus, value) for value in references]
    plus_errors = [error(plus, value) for value in references]
    scores = ([sum(minus_errors) / len(references), sum(plus_errors) / len(references)]
              if references else [None, None])
    ready = bool(references) or bool(reverse)
    selected_minus = ((scores[0] <= scores[1]) != bool(reverse)) if references else False
    votes = [a <= b for a, b in zip(minus_errors, plus_errors)]
    return {
        "baseline_azimuth_deg": azimuth,
        "candidate_minus90_deg": minus, "candidate_plus90_deg": plus,
        "selected_yaw_deg": (minus if selected_minus else plus) if ready else None,
        "selected_offset_deg": (-90.0 if selected_minus else 90.0) if ready else None,
        "mode": ("auto_reverse" if reverse else "auto") if references else ("reverse_forced" if reverse else "auto"),
        "reverse": bool(reverse), "ready": ready,
        "score_minus90_deg": scores[0], "score_plus90_deg": scores[1],
        "consensus": len(set(votes)) == 1 if votes else None,
        "low_margin": abs(scores[0] - scores[1]) < 5.0 if references else False,
        "reason": ("P1/P2 yaw aggregate" if len(set(votes)) <= 1 else "P1/P2 yaw branch disagreement")
                  if references else ("persistent Reverse branch" if reverse else "Pixhawk yaw reference unavailable"),
    }


def campaign_yaw90(sensor_observations):
    """Resolve synthetic campaign yaw; nominal north selects the hemisphere only.

    Existing campaign inputs contain synthetic optical R/P, not measured
    Pixhawk yaw. Declare a nominal 0-degree branch reference rather than
    manufacturing telemetry or choosing the branch with GT/V0 errors.
    R/P observations remain per-camera and unchanged. The resulting yaw
    is shared by both cameras; it is an initializer, never a yaw prior.
    """
    evidence = {
        "schema_version": "simulation_campaign_gps_yaw90_v1",
        "position_source": "synthetic_camera_rtk_optical_centers",
        "input_axis_contract": "world ENU [east,north,up] metres",
        "branch_axis_contract": "NED horizontal [north,east]; clockwise_from_north_deg",
        "branch_reference_source": "declared_nominal_north_not_measured_pixhawk_yaw",
        "branch_reference_yaw_deg": {"left": 0.0, "right": 0.0},
        "runtime_sensor_claim": False,
        "workstation_method_id": "pixhawk_rp_gps_baseline_yaw",
        "ready": False,
    }
    try:
        centers = [sensor_observations["cameras"][side]["optical_center_rtk_position_world_enu_m"]
                   for side in ("left", "right")]
        if any(len(center) != 3 or not all(math.isfinite(float(v)) for v in center)
               for center in centers):
            raise ValueError("camera optical centers must be finite ENU triples")
        delta = [float(centers[1][i]) - float(centers[0][i]) for i in range(3)]
        selection = select_yaw90(north_m=delta[1], east_m=delta[0],
                                 p1_yaw_deg=0.0, p2_yaw_deg=0.0)
        evidence.update(baseline_world_enu_m=delta,
                        baseline_ned_m=[delta[1], delta[0], -delta[2]],
                        selection=selection, ready=selection["ready"])
        yaw = math.radians(selection["selected_yaw_deg"])
        # Signed radians keep yaw close to the nominal start without changing rotation.
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        return {side: yaw for side in ("left", "right")}, evidence
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        evidence["failure_reason"] = str(exc)
        return {"left": None, "right": None}, evidence
