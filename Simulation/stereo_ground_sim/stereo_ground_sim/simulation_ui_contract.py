"""Pure view-model rules for the read-only Simulation research UI."""

from __future__ import annotations

import json
import math

STALE_TIMEOUT_S = 2.0
UNAVAILABLE = "unavailable"
UNSYNCHRONIZED = "N/A — samples not synchronized"

SOURCE_KEYS = (
    "ground_truth",
    "observed_raw",
    "analytic_raw",
    "corrected",
    "attitude_status",
    "observation_status",
    "profile_status",
    "left_image",
    "right_image",
    "ground_truth_range",
    "observed_disparity_px",
    "analytic_disparity_px",
    "pixels",
    "ray_quality",
    "px4_status",
)

PX4_STATUS_TOPIC = "/simulation/px4/status"
PX4_PHASES = (
    "UNAVAILABLE",
    "WAITING",
    "CONNECTED",
    "PRESTREAM",
    "OFFBOARD",
    "ARMING",
    "TAKEOFF",
    "HOVER_1",
    "APPROACH",
    "HOVER_2",
    "LANDING",
    "COMPLETE",
    "ERROR",
)
TOP_BAR_FIELDS = (
    "AUTHORITY",
    "GAZEBO MOTION",
    "PX4 CONNECTION",
    "ARM",
    "MODE",
    "MISSION PHASE",
    "GT SOURCE",
    "OVERALL STATUS",
)
FLIGHT_CONTRACT_FIELDS = (
    "flight_authority",
    "fly_target_in_gazebo",
    "mission_profile",
    "actual_distance_source",
)
EXPECTED_FLIGHT_CONTRACT = {
    "flight_authority": "px4_sitl",
    "fly_target_in_gazebo": False,
    "mission_profile": "manual",
    "actual_distance_source": "gazebo",
}
FLIGHT_CONTRACT_SOURCE = "stereo_distance_node_parameters"
PROFILE_NOT_CONFIGURED_VALUES = {
    "",
    "no_profile",
    "none",
    "disabled",
    "unavailable",
}


def finite_float(value):
    if isinstance(value, (bool, str)):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def disparity_difference_px(observed_disparity_px, analytic_disparity_px):
    """Return observed minus analytic disparity only for finite numeric inputs."""
    observed = finite_float(observed_disparity_px)
    analytic = finite_float(analytic_disparity_px)
    if observed is None or analytic is None:
        return None
    return observed - analytic


def intersection_angle_deg(intersection_angle_rad):
    """Convert the producer's radian ray angle at the display boundary."""
    angle = finite_float(intersection_angle_rad)
    return None if angle is None else math.degrees(angle)


def _profile_is_configured(profile_status):
    if isinstance(profile_status, str):
        try:
            profile_status = json.loads(profile_status)
        except (TypeError, ValueError, json.JSONDecodeError):
            return False
    if not isinstance(profile_status, dict):
        return False
    profile_id = str(profile_status.get("profile_id") or "").strip().lower()
    profile_mode = str(profile_status.get("profile_mode") or "").strip().lower()
    reason = str(profile_status.get("fallback_reason") or "").strip().lower()
    fallback_level = finite_float(profile_status.get("fallback_level"))
    if (
        ("profile_id" in profile_status and profile_id in PROFILE_NOT_CONFIGURED_VALUES)
        or ("profile_mode" in profile_status and profile_mode in PROFILE_NOT_CONFIGURED_VALUES)
    ):
        return False
    if reason and (reason in PROFILE_NOT_CONFIGURED_VALUES or reason == "init"):
        return False
    if profile_id or profile_mode:
        return True
    if fallback_level is not None and fallback_level < 3:
        return True
    return bool(reason)


def detection_display_state(observation_status, side) -> str:
    """Classify detector validity independently from camera stream freshness.

    ``stereo_distance_node`` publishes ``bbox_<side>_valid`` as the primary
    detector contract. Older or partial payloads may omit that field, so the
    finite bbox center is used only as a local fallback. A missing payload or
    missing center remains WAITING rather than being presented as valid.
    """
    if not isinstance(observation_status, dict) or side not in {"left", "right"}:
        return "WAITING"
    prefix = f"bbox_{side}"
    validity = observation_status.get(f"{prefix}_valid")
    if isinstance(validity, bool):
        return "VALID" if validity else "INVALID"
    center_keys = (f"{prefix}_u_px", f"{prefix}_v_px")
    center_present = any(key in observation_status for key in center_keys)
    if all(finite_float(observation_status.get(key)) is not None for key in center_keys):
        return "VALID"
    return "INVALID" if center_present else "WAITING"


def corrected_measurement_state(corrected_source_state, profile_status) -> str:
    """Classify corrected output without conflating missing profile with invalid data."""
    if not _profile_is_configured(profile_status):
        return "NOT CONFIGURED"
    state = str(corrected_source_state or "waiting").strip().lower()
    return {
        "waiting": "WAITING",
        "valid": "VALID",
        "stale": "STALE",
        "invalid": "INVALID",
    }.get(state, "INVALID")


def parse_px4_status(value):
    """Parse a PX4 status JSON value without inventing unavailable fields."""
    try:
        parsed = json.loads(value) if isinstance(value, str) else value
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}
    if not isinstance(parsed, dict) or parsed.get("schema_version") != 1:
        return {}
    return parsed


def px4_contract_state(status):
    """Return ``safe``, ``violation``, or ``unavailable`` for UI display."""
    if not isinstance(status, dict) or not status:
        return "unavailable"
    contract = status.get("flight_contract")
    if not isinstance(contract, dict):
        # Preserve an explicit legacy violation diagnostic, but never treat
        # top-level adapter constants as proof of the live node contract.
        if status.get("flight_authority") not in (None, "px4_sitl") or status.get("gazebo_motion_enabled") not in (None, False):
            return "violation"
        return "unavailable"
    if contract.get("verified") is True and contract.get("source") == FLIGHT_CONTRACT_SOURCE:
        for name, expected in EXPECTED_FLIGHT_CONTRACT.items():
            actual = contract.get(name)
            if isinstance(expected, bool):
                matches = isinstance(actual, bool) and actual is expected
            else:
                matches = isinstance(actual, str) and actual == expected
            if not matches:
                return "violation"
        return "safe"
    error = str(contract.get("error") or "").lower()
    if "violation" in error or contract.get("source") not in (None, FLIGHT_CONTRACT_SOURCE) or any(
        name in contract
        and contract.get(name) is not None
        and (
            (isinstance(expected, bool) and not (isinstance(contract.get(name), bool) and contract.get(name) is expected))
            or (not isinstance(expected, bool) and not (isinstance(contract.get(name), str) and contract.get(name) == expected))
        )
        for name, expected in EXPECTED_FLIGHT_CONTRACT.items()
    ):
        return "violation"
    return "unavailable"


def mission_phase_display(status):
    """Render mission phase with explicit authoritative/inferred provenance."""
    if not isinstance(status, dict):
        return "UNAVAILABLE"
    phase = str(status.get("mission_phase") or "UNAVAILABLE")
    source = status.get("mission_phase_source")
    if phase not in PX4_PHASES:
        return "UNAVAILABLE"
    if source == "authoritative_mission_event" and phase != "UNAVAILABLE":
        return phase
    if source == "inferred_mavlink_state" and phase != "UNAVAILABLE":
        return f"{phase} (INFERRED)"
    return "UNAVAILABLE"


def px4_overall_status(status, source_states=None):
    """Classify the read-only panel status without process-name guesses."""
    contract = px4_contract_state(status)
    if contract == "violation":
        return "ERROR — FLIGHT AUTHORITY CONTRACT VIOLATION"
    if contract == "unavailable":
        return "DEGRADED — FLIGHT AUTHORITY UNVERIFIED"
    if not status.get("px4_connected"):
        return "DEGRADED — PX4 unavailable"
    if status.get("failsafe"):
        return "ERROR — PX4 FAILSAFE"
    states = source_states or {}
    if any(state in {"invalid", "stale"} for state in states.values()):
        return "DEGRADED — source stale or invalid"
    return "VALID — PX4 telemetry connected"


def status_field(status, key, unavailable=UNAVAILABLE):
    """Get a scalar status field while preserving explicit unavailable state."""
    if not isinstance(status, dict):
        return unavailable
    value = status.get(key)
    return unavailable if value is None or value == "" else value


def format_status_vector(vector, keys):
    if not isinstance(vector, dict):
        return UNAVAILABLE
    values = [finite_float(vector.get(key)) for key in keys]
    if any(value is None for value in values):
        return UNAVAILABLE
    return "[" + ", ".join(f"{value:.3f}" for value in values) + "]"


def format_receive_age(received_at, now):
    if received_at is None:
        return UNAVAILABLE
    age = finite_float(now) - finite_float(received_at)
    return UNAVAILABLE if age is None or age < 0.0 else f"{age:.1f} s"


def source_valid(value) -> bool:
    if value is None:
        return False
    if isinstance(value, dict):
        return bool(value)
    if isinstance(value, (list, tuple)):
        return bool(value) and all(finite_float(item) is not None for item in value)
    return finite_float(value) is not None


def source_state(received_at, value, now, threshold=STALE_TIMEOUT_S) -> str:
    if received_at is None:
        return "waiting"
    if not source_valid(value):
        return "invalid"
    return "stale" if now - received_at > threshold else "valid"


def stale_display(value, state, *, unit="", age=None):
    if state == "stale":
        suffix = "unavailable" if age is None else f"{age:.1f} s"
        numeric = "unavailable" if finite_float(value) is None else f"{float(value):.3f}{(' ' + unit) if unit else ''}"
        return f"STALE — last {numeric}; receive age: {suffix}"
    if state in {"waiting", "invalid"}:
        return "unavailable" if state == "invalid" else "WAITING"
    numeric = finite_float(value)
    return "unavailable" if numeric is None else f"{numeric:.3f}{(' ' + unit) if unit else ''}"


def fallback_transition(previous, current):
    """Return exactly one event for a fallback boolean/source transition."""
    def normalize(value):
        if isinstance(value, dict):
            source = value.get("source") or value.get("oracle_source_role")
            flag = value.get("fallback", value.get("oracle_fallback_used", False))
            if flag in (None, False, "False", "false", "unavailable", "None"):
                return False, None
            return True, source
        if value in (None, False, "False", "false", "unavailable", "None"):
            return False, None
        return bool(value), None
    old_flag, old_source = normalize(previous)
    new_flag, new_source = normalize(current)
    if old_flag == new_flag and old_source == new_source:
        return []
    if not old_flag and new_flag:
        return ["Ground Truth fallback started"]
    if old_flag and not new_flag:
        return ["Ground Truth fallback ended"]
    return [f"Ground Truth fallback source changed: {old_source or 'unknown'} → {new_source or 'unknown'}"]


def transition_events(previous, current):
    events = []
    for source in sorted(set(previous) | set(current)):
        before, after = previous.get(source, "waiting"), current.get(source, "waiting")
        if before == after:
            continue
        if before == "waiting" and after == "valid":
            events.append(f"{source}: waiting → valid")
        elif before == "valid" and after == "invalid":
            events.append(f"{source}: valid → invalid")
        elif before == "valid" and after == "stale":
            events.append(f"{source}: valid → stale")
        elif before == "stale" and after == "valid":
            events.append(f"{source}: stale → recovered")
        else:
            events.append(f"{source}: {before} → {after}")
    return events


def synchronized_comparison_errors(truth_range, observed_range, analytic_range, corrected_range, *, synchronized=False):
    """Return comparison values only for an explicitly verified same sample."""
    if not synchronized:
        return {"observed_error": UNSYNCHRONIZED, "analytic_error": UNSYNCHRONIZED, "corrected_error": UNSYNCHRONIZED, "improvement": UNSYNCHRONIZED}
    values = [finite_float(value) for value in (truth_range, observed_range, analytic_range, corrected_range)]
    if any(value is None for value in values):
        return {"observed_error": UNAVAILABLE, "analytic_error": UNAVAILABLE, "corrected_error": UNAVAILABLE, "improvement": UNSYNCHRONIZED}
    truth, observed, analytic, corrected = values
    raw_error = observed - truth
    corrected_error = corrected - truth
    if abs(raw_error) < 1.0e-12:
        improvement = "N/A — raw error is zero"
    else:
        improvement = 1.0 - abs(corrected_error) / abs(raw_error)
    return {"observed_error": raw_error, "analytic_error": analytic - truth, "corrected_error": corrected_error, "improvement": improvement}


def plot_series_for_source(source_key):
    return source_key if source_key in {"ground_truth", "observed_raw", "analytic_raw", "corrected"} else None


def should_append_plot(source_key, changed_key):
    return plot_series_for_source(source_key) == changed_key


def stale_marker_action(state):
    return "DELETE" if state in {"stale", "invalid", "waiting"} else "ADD"


__all__ = [
    "STALE_TIMEOUT_S", "UNAVAILABLE", "UNSYNCHRONIZED", "SOURCE_KEYS", "PX4_STATUS_TOPIC", "PX4_PHASES", "TOP_BAR_FIELDS", "FLIGHT_CONTRACT_FIELDS", "EXPECTED_FLIGHT_CONTRACT", "FLIGHT_CONTRACT_SOURCE", "finite_float",
    "disparity_difference_px", "intersection_angle_deg", "corrected_measurement_state",
    "source_valid", "source_state", "stale_display", "fallback_transition", "transition_events", "synchronized_comparison_errors",
    "plot_series_for_source", "should_append_plot", "stale_marker_action", "parse_px4_status", "px4_contract_state", "px4_overall_status", "mission_phase_display", "status_field", "format_status_vector", "format_receive_age",
]
