"""Pure GNSS NED/body-FRD and baseline-center display geometry.

Position WGS84 heights and attitude body-FRD->world-NED remain distinct;
existing geometry functions retain their published frame and sign contracts.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass

from Workstation.evidence_views.values import _age, _integer, _number
from Workstation.workstation_core.attitude_geometry import relative_attitude, world_ned_vector_to_body_frd
from Workstation.workstation_core.coordinate_contract import BODY_FRAME
from Workstation.workstation_core.drone_pixhawk_geometry import (
    calculate_horizontal_triangle_angles,
    subtract_vectors,
    vector_range_m,
)
from Workstation.workstation_core.gnss_baseline_center import (
    DEFAULT_MAXIMUM_GNSS_AGE_S,
    DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S,
    estimate_gnss_baseline_center_frd,
)
from Workstation.workstation_core.physical_alignment import (
    angle_error_deg,
    calculate_baseline_yaw_alignment,
    normalize_compass_angle_deg,
)
from Workstation.workstation_core.pixhawk_link.policy import MAVLINK_ATTITUDE_STALE_S


@dataclass(frozen=True)
class InvalidGpsInput:
    """Preserve malformed-vs-absent legacy input evidence without its live object."""

    timestamp_s: float | None


BASELINE_FIELDS = ("fix_quality", "north_m", "east_m", "down_m", "baseline_m", "left_age_s", "right_age_s", "warnings")
BASELINE_PROJECTION_FIELDS = ("forward_m", "right_m", "down_m", "attitude_age_s", "target_frame", "axis_contract")
BASELINE_QUALITY_FIELDS = ("sample_count", "rtk_fixed_ratio", "baseline_length_std_m", "timestamp_delta_ms", "reasons")


def project_baseline(*, baseline, projection, quality, selected_left, selected_right, left_source, right_source):
    quality = quality or {}
    selected_left, selected_right = selected_left or {}, selected_right or {}
    quality_values: dict[str, object] = {
        "sample_count": _integer(quality.get("sample_count", None)),
        "fixed_ratio": _number(quality.get("rtk_fixed_ratio", None)),
        "length_std_m": _number(quality.get("baseline_length_std_m", None)),
        "timestamp_delta_ms": _number(quality.get("timestamp_delta_ms", None)),
        "reasons": tuple(quality.get("reasons", ()) or ()),
    }
    projection_values: dict[str, object] = {
        "p1_body_frd_state": "WAITING",
        "p1_body_frd_forward_m": None,
        "p1_body_frd_right_m": None,
        "p1_body_frd_down_m": None,
        "p1_body_frd_attitude_age_s": None,
        "p1_body_frd_source": "P1 attitude-derived projection; waiting for P1 body FRD attitude",
        "p1_body_frd_frame": BODY_FRAME,
        "p1_body_frd_axis_contract": "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
    }
    if projection is not None:
        projection_values.update(
            {
                "p1_body_frd_state": "READY",
                "p1_body_frd_forward_m": _number(projection.get("forward_m", None)),
                "p1_body_frd_right_m": _number(projection.get("right_m", None)),
                "p1_body_frd_down_m": _number(projection.get("down_m", None)),
                "p1_body_frd_attitude_age_s": _number(projection.get("attitude_age_s", None)),
                "p1_body_frd_source": "BodyFrdBaselineProjection from raw NED + P1 attitude",
                "p1_body_frd_frame": str(projection.get("target_frame", BODY_FRAME)),
                "p1_body_frd_axis_contract": str(
                    projection.get("axis_contract", "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres")
                ),
            }
        )
    if baseline is None:
        return {
            "state": "NO DATA",
            "source": "RtkBaselineEstimate from left/right GpsPositionState",
            "axis_contract": "NED = north / east / down; metres",
            "quality": quality_values,
            "left_source": left_source,
            "right_source": right_source,
            "left_fix_type": _integer(selected_left.get("fix_type", None)),
            "right_fix_type": _integer(selected_right.get("fix_type", None)),
            **projection_values,
        }
    return {
        "state": str(baseline.get("fix_quality", "UNKNOWN") or "UNKNOWN"),
        "north_m": _number(baseline.get("north_m", None)),
        "east_m": _number(baseline.get("east_m", None)),
        "down_m": _number(baseline.get("down_m", None)),
        "length_m": _number(baseline.get("baseline_m", None)),
        "left_age_s": _number(baseline.get("left_age_s", None)),
        "right_age_s": _number(baseline.get("right_age_s", None)),
        "warnings": tuple(baseline.get("warnings", ()) or ()),
        "source": "RtkBaselineEstimate from left/right GpsPositionState",
        "axis_contract": "NED = north / east / down; metres",
        "position_frame": "GPS/RTK WGS84 delta projected to local NED; not camera or FRD",
        "quality": quality_values,
        "left_source": left_source,
        "right_source": right_source,
        "left_fix_type": _integer(selected_left.get("fix_type", None)),
        "right_fix_type": _integer(selected_right.get("fix_type", None)),
        **projection_values,
    }


def project_baseline_yaw(*, baseline, attitudes, gps, sources, maximum_gps_age_s):
    def finite_number(value: object) -> float | None:
        number = _number(value)
        return number if number is not None and math.isfinite(number) else None

    def freshness_state(value: float | None, age_s: float | None, maximum_age_s: float) -> str:
        if value is None:
            return "WAITING"
        if age_s is None or not math.isfinite(age_s) or age_s > maximum_age_s:
            return "STALE"
        return "CURRENT"

    ned_values = tuple(finite_number(baseline.get(key)) for key in ("north_m", "east_m", "down_m"))
    raw_yaws = {side: finite_number(attitudes[side].get("yaw_deg")) for side in ("left", "right")}
    compass_yaws = {
        side: (None if raw_yaws[side] is None else normalize_compass_angle_deg(raw_yaws[side], f"{side}_yaw_deg"))
        for side in ("left", "right")
    }
    geometry = None
    baseline_reason = ""
    if all(value is not None for value in ned_values):
        geometry = calculate_baseline_yaw_alignment(
            ned_values,  # type: ignore[arg-type]
            p1_yaw_deg=raw_yaws["left"],
            p2_yaw_deg=raw_yaws["right"],
        )
        if geometry is None:
            baseline_reason = "P1→P2 horizontal baseline direction is undefined"
    else:
        baseline_reason = "P1→P2 NED baseline waiting"

    gps_ages = {side: finite_number(gps[side].get("age_s")) for side in ("left", "right")}
    gps_positions_present = {
        side: all(
            finite_number(gps[side].get(key)) is not None for key in ("latitude_deg", "longitude_deg", "altitude_m")
        )
        for side in ("left", "right")
    }
    gps_states = {
        side: freshness_state(
            1.0 if gps_positions_present[side] else None,
            gps_ages[side],
            maximum_gps_age_s,
        )
        for side in ("left", "right")
    }
    baseline_state = "WAITING"
    if geometry is not None:
        baseline_state = "CURRENT" if all(state == "CURRENT" for state in gps_states.values()) else "STALE"

    attitude_ages = {side: finite_number(attitudes[side].get("age_s")) for side in ("left", "right")}
    side_states = {
        side: freshness_state(
            compass_yaws[side],
            attitude_ages[side],
            MAVLINK_ATTITUDE_STALE_S,
        )
        for side in ("left", "right")
    }
    if geometry is None:
        state = "WAITING"
    elif baseline_state == "STALE":
        state = "STALE"
    else:
        current_count = sum(value == "CURRENT" for value in side_states.values())
        if current_count == 2:
            state = "CURRENT"
        elif current_count == 1:
            state = "PARTIAL"
        elif any(value == "STALE" for value in side_states.values()):
            state = "STALE"
        else:
            state = "WAITING"

    reasons: list[str] = []
    if baseline_reason:
        reasons.append(baseline_reason)
    elif baseline_state == "STALE":
        reasons.append("selected P1/P2 GNSS baseline is stale")
    for side, label in (("left", "P1"), ("right", "P2")):
        if side_states[side] != "CURRENT":
            reasons.append(f"{label} ATTITUDE {side_states[side].lower()}")

    gps_timestamps = {side: finite_number(gps[side].get("timestamp_s")) for side in ("left", "right")}
    gps_pair_delta_ms = None
    if all(value is not None for value in gps_timestamps.values()):
        gps_pair_delta_ms = (
            abs(
                gps_timestamps["right"] - gps_timestamps["left"]  # type: ignore[operator]
            )
            * 1000.0
        )
    baseline_age_values = [value for value in gps_ages.values() if value is not None]
    baseline_age_s = max(baseline_age_values) if baseline_age_values else None

    return {
        "state": state,
        "reason": "; ".join(reasons),
        "baseline_state": baseline_state,
        "p1_state": side_states["left"],
        "p2_state": side_states["right"],
        "baseline_azimuth_deg": (None if geometry is None else geometry.baseline_azimuth_deg),
        "p1_yaw_deg": compass_yaws["left"],
        "p2_yaw_deg": compass_yaws["right"],
        "baseline_p1_deg": None if geometry is None else geometry.baseline_p1_deg,
        "baseline_p2_deg": None if geometry is None else geometry.baseline_p2_deg,
        "p2_minus_p1_deg": (
            geometry.p2_minus_p1_deg
            if geometry is not None
            else (
                None
                if raw_yaws["left"] is None or raw_yaws["right"] is None
                else angle_error_deg(raw_yaws["left"], raw_yaws["right"])
            )
        ),
        "baseline_age_s": baseline_age_s,
        "p1_attitude_age_s": attitude_ages["left"],
        "p2_attitude_age_s": attitude_ages["right"],
        "gps_pair_delta_ms": gps_pair_delta_ms,
        "maximum_gps_age_s": maximum_gps_age_s,
        "attitude_stale_age_s": MAVLINK_ATTITUDE_STALE_S,
        "p1_gps_source": sources["left"],
        "p2_gps_source": sources["right"],
        "p1_fix_type": gps["left"].get("fix_type"),
        "p2_fix_type": gps["right"].get("fix_type"),
        "p1_rtk_label": gps["left"].get("rtk_label", gps["left"].get("state", "NO DATA")),
        "p2_rtk_label": gps["right"].get("rtk_label", gps["right"].get("state", "NO DATA")),
        "p1_gps_age_s": gps_ages["left"],
        "p2_gps_age_s": gps_ages["right"],
        "baseline_quality": baseline.get("state", "NO DATA"),
        "baseline_length_m": finite_number(baseline.get("length_m")),
        "source": "selected P1/P2 GpsPositionState + P1/P2 MAVLink ATTITUDE",
        "clock": "latest app receive monotonic snapshots; not simultaneous measurement",
        "frame_contract": "P1→P2 horizontal local NED; P1/P2 body FRD yaw in world NED",
        "angle_contract": (
            "Baseline-Pn = wrap180(azimuth(P1→P2) - Pn yaw), Pn=P1/P2; "
            "+ is clockwise/right, - is counter-clockwise/left"
        ),
    }


def project_drone_geometry(*, p1, p2, drone, p1_source, p2_source, attitudes, drone_frozen, now):
    estimate = estimate_gnss_baseline_center_frd(
        p1,
        p2,
        drone,
        now_s=now,
        maximum_age_s=DEFAULT_MAXIMUM_GNSS_AGE_S,
        maximum_timestamp_spread_s=DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S,
    )
    geometry = estimate.geometry
    geometry_state = "WAITING" if geometry is None else "STALE" if estimate.state == "STALE" else "CURRENT"

    def finite_age(state: object | None) -> float | None:
        if state is None:
            return None
        age = _age(getattr(state, "timestamp_s", None), now)
        return age if age is not None and math.isfinite(age) else None

    p1_age = finite_age(p1)
    p2_age = finite_age(p2)
    drone_age = finite_age(drone)
    gps_ages = tuple(age for age in (p1_age, p2_age, drone_age) if age is not None)
    gps_age = max(gps_ages) if gps_ages else None

    def body_attitude_state(data: Mapping[str, object]) -> str:
        q = data.get("q_wxyz")
        age = data.get("age_s")
        if not isinstance(q, (tuple, list)) or len(q) != 4:
            return "WAITING"
        try:
            finite_q = all(math.isfinite(float(value)) for value in q)
            finite_age_value = age is not None and math.isfinite(float(age))
        except (TypeError, ValueError):
            return "WAITING"
        if not finite_q or not finite_age_value:
            return "WAITING"
        return "STALE" if float(age) > MAVLINK_ATTITUDE_STALE_S else "CURRENT"

    p1_attitude_state = body_attitude_state(attitudes["p1"])
    p2_attitude_state = body_attitude_state(attitudes["p2"])

    def vector_row(
        key: str,
        label: str,
        vector: tuple[float, float, float] | None,
        *,
        state: str,
        frame: str,
        age_s: float | None,
        source: str,
        reason: str,
    ) -> dict[str, object]:
        if vector is None:
            return {
                "key": key,
                "label": label,
                "state": state,
                "forward_m": None,
                "right_m": None,
                "down_m": None,
                "range_m": None,
                "frame": frame,
                "age_s": age_s,
                "source": source,
                "reason": reason,
            }
        return {
            "key": key,
            "label": label,
            "state": state,
            "forward_m": vector[0],
            "right_m": vector[1],
            "down_m": vector[2],
            "range_m": vector_range_m(vector),
            "frame": frame,
            "age_s": age_s,
            "source": source,
            "reason": reason,
        }

    def combined_body_state(base_state: str, attitude_state: str) -> str:
        if base_state == "WAITING":
            return "WAITING"
        if base_state == "STALE" or attitude_state == "STALE":
            return "STALE"
        if attitude_state == "CURRENT":
            return "CURRENT"
        return "WAITING"

    vectors: list[dict[str, object]] = []
    if geometry is None:
        midpoint_vector = None
        p1_vector_ned = None
        p2_vector_ned = None
    else:
        midpoint_vector = geometry.vector_m
        p1_vector_ned = geometry.drone_from_p1_ned_m
        p2_vector_ned = subtract_vectors(
            geometry.drone_from_p1_ned_m,
            geometry.baseline_ned_m,
        )

    midpoint_reason = (
        estimate.reason if geometry is None or geometry_state != "CURRENT" else "Three GNSS positions are current"
    )
    vectors.append(
        vector_row(
            "baseline_center",
            "Dr from Baseline Center",
            midpoint_vector,
            state=geometry_state,
            frame=(
                str(getattr(geometry, "frame", "gnss_baseline_center_local_level_frd"))
                if geometry is not None
                else "gnss_baseline_center_local_level_frd"
            ),
            age_s=gps_age,
            source="selected P1/P2 GNSS + Drone GNSS; midpoint geometry",
            reason=midpoint_reason,
        )
    )

    def transformed_body_vector(
        vector_ned: tuple[float, float, float] | None,
        attitude: Mapping[str, object],
    ) -> tuple[float, float, float] | None:
        if vector_ned is None:
            return None
        q = attitude.get("q_wxyz")
        if not isinstance(q, (tuple, list)) or len(q) != 4:
            return None
        try:
            return world_ned_vector_to_body_frd(vector_ned, q)
        except (TypeError, ValueError):
            return None

    p1_body_vector = transformed_body_vector(p1_vector_ned, attitudes["p1"])
    p2_body_vector = transformed_body_vector(p2_vector_ned, attitudes["p2"])
    p1_body_state = combined_body_state(geometry_state, p1_attitude_state)
    p2_body_state = combined_body_state(geometry_state, p2_attitude_state)
    p1_body_reason = estimate.reason if p1_attitude_state in {"CURRENT", "STALE"} else "P1 ATTITUDE waiting or invalid"
    p2_body_reason = estimate.reason if p2_attitude_state in {"CURRENT", "STALE"} else "P2 ATTITUDE waiting or invalid"
    vectors.append(
        vector_row(
            "p1",
            "Dr from P1",
            p1_body_vector,
            state=p1_body_state,
            frame=f"{BODY_FRAME} (P1 body)",
            age_s=max(
                (age for age in (gps_age, attitudes["p1"].get("age_s")) if age is not None),
                default=None,
            ),
            source="P1→Dr GNSS vector + P1 MAVLink ATTITUDE",
            reason=p1_body_reason,
        )
    )
    vectors.append(
        vector_row(
            "p2",
            "Dr from P2",
            p2_body_vector,
            state=p2_body_state,
            frame=f"{BODY_FRAME} (P2 body)",
            age_s=max(
                (age for age in (gps_age, attitudes["p2"].get("age_s")) if age is not None),
                default=None,
            ),
            source="P2→Dr GNSS vector + P2 MAVLink ATTITUDE",
            reason=p2_body_reason,
        )
    )

    angles = None
    if geometry is not None:
        angles = calculate_horizontal_triangle_angles(
            (0.0, 0.0, 0.0),
            geometry.baseline_ned_m,
            geometry.drone_from_p1_ned_m,
        )

    def angle_row(
        key: str,
        label: str,
        value: float | None,
        *,
        vertex: str,
    ) -> dict[str, object]:
        state = geometry_state if value is not None else "WAITING"
        reason = (
            estimate.reason
            if value is not None
            else "horizontal ray is undefined"
            if geometry is not None
            else estimate.reason
        )
        return {
            "key": key,
            "label": label,
            "angle_deg": value,
            "state": state,
            "vertex": vertex,
            "age_s": gps_age,
            "source": "P1/P2/Drone GNSS horizontal positions",
            "frame": "horizontal local NED (North/East only)",
            "reason": reason,
        }

    angles_rows = (
        angle_row(
            "dr_p1_p2",
            "∠Dr-P1-P2",
            None if angles is None else angles.at_p1_deg,
            vertex="P1",
        ),
        angle_row(
            "dr_p2_p1",
            "∠Dr-P2-P1",
            None if angles is None else angles.at_p2_deg,
            vertex="P2",
        ),
    )

    angle_values = tuple(row.get("angle_deg") for row in angles_rows)
    if geometry_state == "STALE":
        overall_state = "STALE"
    elif geometry_state != "CURRENT":
        overall_state = geometry_state
    else:
        current_attitudes = sum(state == "CURRENT" for state in (p1_attitude_state, p2_attitude_state))
        if current_attitudes == 2:
            overall_state = "CURRENT" if all(value is not None for value in angle_values) else "PARTIAL"
        elif current_attitudes == 1:
            overall_state = "PARTIAL"
        elif any(state == "STALE" for state in (p1_attitude_state, p2_attitude_state)):
            overall_state = "STALE"
        else:
            overall_state = "WAITING"

    reasons: list[str] = []
    if drone_frozen is not None:
        # A vehicle whose GNSS keeps repeating one position produces a
        # triangle that only looks current: age resets on every packet
        # while the vertex never moves. Say so ahead of every other reason.
        overall_state = "FROZEN"
        cause = "autopilot이 같은 fix 재전송" if drone_frozen.get("utc_frozen") else "수신기 위치 출력 정지"
        reasons.append(
            f"Drone GNSS 위치 고정 ×{_integer(drone_frozen.get('packets'))} ({cause}); "
            "각도/벡터는 정지된 드론 위치 기준"
        )
    if estimate.reason and geometry_state != "CURRENT":
        reasons.append(estimate.reason)
    if p1_attitude_state not in {"CURRENT"}:
        reasons.append(f"P1 ATTITUDE {p1_attitude_state.lower()}")
    if p2_attitude_state not in {"CURRENT"}:
        reasons.append(f"P2 ATTITUDE {p2_attitude_state.lower()}")
    if geometry_state == "CURRENT" and any(value is None for value in angle_values):
        reasons.append("one or both horizontal triangle angles are undefined")
    return {
        "state": overall_state,
        "drone_position_frozen": drone_frozen is not None,
        "reason": "; ".join(dict.fromkeys(reasons)) or "all geometry inputs current",
        "vectors": tuple(vectors),
        "angles": angles_rows,
        "p1_gps_source": p1_source,
        "p2_gps_source": p2_source,
        "p1_gps_age_s": p1_age,
        "p2_gps_age_s": p2_age,
        "drone_gps_age_s": drone_age,
        "source": "selected P1/P2 GNSS + Drone GNSS + P1/P2 MAVLink ATTITUDE",
        "clock": "latest app receive monotonic snapshots; not simultaneous measurement",
        "frame_contract": (
            "midpoint→Dr is GNSS baseline-center local-level FRD; P1/P2→Dr are respective PX4 body FRD vectors"
        ),
        "angle_contract": (
            "∠Dr-P1-P2 and ∠Dr-P2-P1 use horizontal North/East only; unsigned internal angles in [0,180] degrees"
        ),
    }


def project_physical_alignment(
    *, quality_mapping, left_q, right_q, baseline, gps, sampling_active, sample_count, iterations
):
    relative_rpy: tuple[float, float, float] | None = None
    if left_q is not None and right_q is not None:
        try:
            relative = relative_attitude(left_q, right_q)
            relative_rpy = (relative.roll_deg, relative.pitch_deg, relative.yaw_deg)
        except (AttributeError, TypeError, ValueError):
            relative_rpy = None
    current_frd = tuple(
        baseline.get(key)
        for key in (
            "p1_body_frd_forward_m",
            "p1_body_frd_right_m",
            "p1_body_frd_down_m",
        )
    )
    if not all(value is not None for value in current_frd):
        current_frd = None
    return {
        "state": "LIVE"
        if relative_rpy is not None and baseline.get("state") not in {"NO DATA", "UNKNOWN"}
        else "WAITING",
        "sampling_active": sampling_active,
        "sample_count": sample_count,
        "current_baseline_ned_m": tuple(baseline.get(key) for key in ("north_m", "east_m", "down_m")),
        "current_baseline_frd_m": current_frd,
        "relative_rpy_deg": relative_rpy,
        "fix_type": {side: gps[side].get("fix_type") for side in ("left", "right")},
        "quality": quality_mapping,
        "iterations": tuple(iterations),
        "physical_adjustment_only": True,
        "coordinate_contract": (
            "raw baseline NED [north,east,down] is preserved; P1→P2 FRD [forward,right,down] "
            "is an attitude-derived observation; ΔR/P/Y is [roll,pitch,yaw]"
        ),
    }
