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


def finite_float(value):
    if isinstance(value, (bool, str)):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


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
    if status.get("flight_authority") != "px4_sitl" or status.get("gazebo_motion_enabled") is not False:
        return "violation"
    return "safe"


def px4_overall_status(status, source_states=None):
    """Classify the read-only panel status without process-name guesses."""
    contract = px4_contract_state(status)
    if contract == "violation":
        return "ERROR — FLIGHT AUTHORITY CONTRACT VIOLATION"
    if contract == "unavailable":
        return "WAITING — PX4 status unavailable"
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
    "STALE_TIMEOUT_S", "UNAVAILABLE", "UNSYNCHRONIZED", "SOURCE_KEYS", "PX4_STATUS_TOPIC", "PX4_PHASES", "TOP_BAR_FIELDS", "finite_float",
    "source_valid", "source_state", "stale_display", "fallback_transition", "transition_events", "synchronized_comparison_errors",
    "plot_series_for_source", "should_append_plot", "stale_marker_action", "parse_px4_status", "px4_contract_state", "px4_overall_status", "status_field", "format_status_vector", "format_receive_age",
]
