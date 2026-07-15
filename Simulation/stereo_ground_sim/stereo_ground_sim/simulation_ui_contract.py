"""Pure view-model rules for the read-only Simulation research UI."""

from __future__ import annotations

import json
import math
import copy
import threading
import time
from dataclasses import dataclass

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
PROFILE_NOT_CONFIGURED = "NOT_CONFIGURED"
PROFILE_ACTIVE = "ACTIVE"
PROFILE_UNKNOWN_INVALID = "UNKNOWN_INVALID"


@dataclass(frozen=True)
class RecordingReadiness:
    """Immutable recording gate result shared by the two Simulation UIs."""

    start_allowed: bool
    formal_eligible: bool
    reasons: tuple[str, ...]

    def as_dict(self):
        return {
            "start_allowed": bool(self.start_allowed),
            "formal_eligible": bool(self.formal_eligible),
            "reasons": list(self.reasons),
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


def latest_unsynchronized_disparity_difference_px(observed_disparity_px, analytic_disparity_px):
    """Return a receive-time diagnostic, never a formal same-sample error."""
    return disparity_difference_px(observed_disparity_px, analytic_disparity_px)


def intersection_angle_deg(intersection_angle_rad):
    """Convert the producer's radian ray angle at the display boundary."""
    angle = finite_float(intersection_angle_rad)
    return None if angle is None else math.degrees(angle)


def profile_contract_state(profile_status):
    """Classify a profile payload without treating fallback metadata as a profile.

    ``fallback_level`` and ``fallback_reason`` describe how a producer
    arrived at its current state; they are not an active profile contract.
    Only an explicit string ``profile_id`` or ``profile_mode`` can establish
    ``ACTIVE``.  Explicit no-profile values remain a valid ``NOT_CONFIGURED``
    state, while malformed or metadata-only payloads are ``UNKNOWN_INVALID``.
    """
    if isinstance(profile_status, str):
        try:
            profile_status = json.loads(profile_status)
        except (TypeError, ValueError, json.JSONDecodeError):
            return PROFILE_UNKNOWN_INVALID
    if not isinstance(profile_status, dict) or not profile_status:
        return PROFILE_UNKNOWN_INVALID

    explicit_fields = [
        name for name in ("profile_id", "profile_mode", "mode") if name in profile_status
    ]
    normalized_fields = {}
    for name in explicit_fields:
        value = profile_status.get(name)
        if value is None or not isinstance(value, str):
            return PROFILE_UNKNOWN_INVALID
        normalized_fields[name] = value.strip().lower()

    active_values = [
        value
        for value in normalized_fields.values()
        if value not in PROFILE_NOT_CONFIGURED_VALUES
    ]
    not_configured_values = [
        value
        for value in normalized_fields.values()
        if value in PROFILE_NOT_CONFIGURED_VALUES
    ]
    if active_values and not_configured_values:
        return PROFILE_UNKNOWN_INVALID
    if active_values:
        return PROFILE_ACTIVE

    status = profile_status.get("status")
    if status is not None and not isinstance(status, str):
        return PROFILE_UNKNOWN_INVALID
    normalized_status = str(status or "").strip().lower()
    reason = str(profile_status.get("fallback_reason") or "").strip().lower()
    if explicit_fields and len(active_values) == 0:
        return PROFILE_NOT_CONFIGURED
    if (
        (normalized_status and normalized_status in PROFILE_NOT_CONFIGURED_VALUES)
        or (reason and reason in PROFILE_NOT_CONFIGURED_VALUES)
    ):
        return PROFILE_NOT_CONFIGURED
    return PROFILE_UNKNOWN_INVALID


def _profile_is_configured(profile_status):
    """Backward-compatible boolean wrapper for the explicit profile contract."""
    return profile_contract_state(profile_status) == PROFILE_ACTIVE


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
    profile_state = profile_contract_state(profile_status)
    if profile_state == PROFILE_NOT_CONFIGURED:
        return "NOT CONFIGURED"
    if profile_state == PROFILE_UNKNOWN_INVALID:
        return PROFILE_UNKNOWN_INVALID
    state = str(corrected_source_state or "waiting").strip().lower()
    return {
        "waiting": "WAITING",
        "valid": "VALID",
        "stale": "STALE",
        "invalid": "INVALID",
    }.get(state, "INVALID")


def state_display_value(value, state, *, unit=""):
    """Format one numeric field without replaying invalid or stale values."""
    normalized = str(state or "waiting").strip().lower()
    if normalized != "valid":
        return "—"
    number = finite_float(value)
    return "—" if number is None else f"{number:.3f}{(' ' + unit) if unit else ''}"


def _status_payload(value):
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (TypeError, ValueError, json.JSONDecodeError):
            return {}
    return value if isinstance(value, dict) else {}


def _source_is_fresh_valid(source_states, key):
    return str((source_states or {}).get(key, "waiting")).strip().lower() == "valid"


def _oracle_role(statuses):
    for status in statuses:
        payload = _status_payload(status)
        role = payload.get("oracle_source_role")
        if role not in (None, ""):
            return str(role).strip().lower(), payload
    return "", {}


def _is_gazebo_oracle_role(role):
    normalized = str(role or "").strip().lower()
    if not normalized or normalized in {"unavailable", "none", "unknown"}:
        return False
    return normalized in {
        "target_reference_link",
        "gazebo_oracle",
        "gazebo_target_reference_link",
    } or ("gazebo" in normalized and "px4" not in normalized)


def source_state_from_monotonic_ns(received_monotonic_ns, value, now_monotonic_ns, threshold_s=STALE_TIMEOUT_S):
    """Classify a source using only receive-time monotonic nanoseconds.

    ROS/simulation clock pause and reset cannot make an old source appear
    fresh because neither value participates in this decision.
    """
    if received_monotonic_ns in (None, ""):
        return "waiting"
    if not source_valid(value):
        return "invalid"
    try:
        received = int(received_monotonic_ns)
        now = int(now_monotonic_ns)
        threshold_ns = max(0, int(float(threshold_s) * 1.0e9))
    except (TypeError, ValueError, OverflowError):
        return "invalid"
    if now < received or now - received > threshold_ns:
        return "stale"
    return "valid"


_READINESS_SOURCE_KEYS = (
    "ground_truth",
    "observed_raw",
    "analytic_raw",
    "corrected",
    "attitude_status",
    "observation_status",
    "profile_status",
    "left_image",
    "right_image",
    "px4_status",
)


class RecordingReadinessModel:
    """Thread-independent readiness model shared by both Simulation panels."""

    def __init__(
        self,
        *,
        mode="full",
        observation_mode="both",
        parameter_snapshot_status="pending",
        monotonic_clock_ns=time.monotonic_ns,
    ):
        self.mode = str(mode or "full")
        self.observation_mode = str(observation_mode or "both")
        self.parameter_snapshot_status = parameter_snapshot_status
        self._monotonic_clock_ns = monotonic_clock_ns
        self._lock = threading.RLock()
        self._payloads = {}
        self._receive_monotonic_ns = {}
        self._receive_ros_time_ns = {}

    def set_mode(self, mode, observation_mode=None):
        with self._lock:
            self.mode = str(mode or "full")
            if observation_mode is not None:
                self.observation_mode = str(observation_mode or "both")

    def set_parameter_snapshot_status(self, status):
        with self._lock:
            self.parameter_snapshot_status = status

    @staticmethod
    def _normalize_ns(value):
        if value in (None, ""):
            return None
        try:
            return int(value)
        except (TypeError, ValueError, OverflowError):
            return None

    def update(self, key, value, *, receive_ros_time_ns=None, receive_monotonic_ns=None):
        key = str(key)
        monotonic_ns = self._normalize_ns(
            self._monotonic_clock_ns() if receive_monotonic_ns is None else receive_monotonic_ns
        )
        with self._lock:
            self._payloads[key] = copy.deepcopy(value)
            self._receive_monotonic_ns[key] = monotonic_ns
            self._receive_ros_time_ns[key] = self._normalize_ns(receive_ros_time_ns)

    def set_source_state(
        self,
        key,
        state,
        value=None,
        *,
        receive_ros_time_ns=None,
        receive_monotonic_ns=None,
    ):
        """Test/fixture-friendly update that still stores a monotonic receipt."""
        key = str(key)
        receive = self._normalize_ns(
            self._monotonic_clock_ns() if receive_monotonic_ns is None else receive_monotonic_ns
        )
        with self._lock:
            self._payloads[key] = copy.deepcopy(value)
            self._receive_monotonic_ns[key] = receive
            self._receive_ros_time_ns[key] = self._normalize_ns(receive_ros_time_ns)
            if str(state).lower() == "waiting":
                self._receive_monotonic_ns.pop(key, None)
                self._receive_ros_time_ns.pop(key, None)

    def readiness(self, *, now_monotonic_ns=None):
        now = int(self._monotonic_clock_ns() if now_monotonic_ns is None else now_monotonic_ns)
        with self._lock:
            mode = self.mode
            observation_mode = self.observation_mode
            parameter_snapshot_status = self.parameter_snapshot_status
            payloads = copy.deepcopy(self._payloads)
            receive_monotonic_ns = dict(self._receive_monotonic_ns)
        states = {
            key: source_state_from_monotonic_ns(
                receive_monotonic_ns.get(key),
                payloads.get(key),
                now,
            )
            for key in _READINESS_SOURCE_KEYS
        }
        observation = _status_payload(payloads.get("observation_status"))
        return evaluate_recording_readiness(
            mode=mode,
            observation_mode=observation.get("observation_mode", observation_mode),
            source_states=states,
            px4_status=payloads.get("px4_status"),
            observation_status=payloads.get("observation_status"),
            attitude_status=payloads.get("attitude_status"),
            profile_status=payloads.get("profile_status"),
            parameter_snapshot_status=parameter_snapshot_status,
        )

    def payload_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._payloads)

    def receive_monotonic_snapshot(self):
        with self._lock:
            return dict(self._receive_monotonic_ns)

    def receive_ros_time_snapshot(self):
        with self._lock:
            return dict(self._receive_ros_time_ns)


def evaluate_recording_readiness(
    *,
    mode,
    observation_mode,
    source_states,
    px4_status,
    observation_status,
    attitude_status,
    profile_status,
    parameter_snapshot_status=None,
):
    """Evaluate the pure start/formal gate for lite, full, and debug runs.

    ``source_states`` must use the panel's ``waiting|valid|invalid|stale``
    values. The function intentionally does not infer a sample timestamp from
    receive time; it only gates freshness and validity.
    """
    normalized_mode = str(mode or "full").strip().lower()
    normalized_observation = str(observation_mode or "both").strip().lower()
    if normalized_observation not in {"bbox", "ideal_projection", "both"}:
        normalized_observation = "both"
    reasons = []

    if normalized_mode == "debug":
        reasons.append("DEBUG — NON-FORMAL")
        return RecordingReadiness(True, False, tuple(reasons))

    states = source_states or {}
    px4 = _status_payload(px4_status)
    observation = _status_payload(observation_status)
    attitude = _status_payload(attitude_status)
    profile = _status_payload(profile_status)

    if not _source_is_fresh_valid(states, "px4_status"):
        reasons.append(f"PX4 status {str(states.get('px4_status', 'waiting')).upper()}")
    if px4_contract_state(px4) != "safe":
        reasons.append("flight contract is not SAFE")
    if px4.get("px4_connected") is not True:
        reasons.append("PX4 is not connected")
    if not _source_is_fresh_valid(states, "ground_truth"):
        reasons.append(f"Ground Truth {str(states.get('ground_truth', 'waiting')).upper()}")

    role, role_payload = _oracle_role((observation, attitude))
    if not _is_gazebo_oracle_role(role):
        reasons.append("Ground Truth source is not Gazebo oracle")
    if role_payload.get("oracle_valid") is not True:
        reasons.append("Ground Truth oracle is not valid")
    if role_payload.get("oracle_fallback_used") is not False:
        reasons.append("Ground Truth fallback is active")

    for key, label in (
        ("left_image", "left image"),
        ("right_image", "right image"),
        ("attitude_status", "attitude status"),
        ("observation_status", "observation status"),
        ("profile_status", "profile status"),
    ):
        if not _source_is_fresh_valid(states, key):
            reasons.append(f"{label} {str(states.get(key, 'waiting')).upper()}")

    if normalized_observation in {"bbox", "both"}:
        if not _source_is_fresh_valid(states, "observed_raw"):
            reasons.append(f"observed raw {str(states.get('observed_raw', 'waiting')).upper()}")
        for side in ("left", "right"):
            if detection_display_state(observation, side) != "VALID":
                reasons.append(f"{side} detection {detection_display_state(observation, side)}")
    if normalized_observation in {"ideal_projection", "both"}:
        if not _source_is_fresh_valid(states, "analytic_raw"):
            reasons.append(f"analytic raw {str(states.get('analytic_raw', 'waiting')).upper()}")

    profile_state = profile_contract_state(profile)
    if _source_is_fresh_valid(states, "profile_status"):
        if profile_state == PROFILE_UNKNOWN_INVALID:
            reasons.append("profile contract UNKNOWN_INVALID")
        elif profile_state == PROFILE_ACTIVE and not _source_is_fresh_valid(states, "corrected"):
            reasons.append(f"corrected {str(states.get('corrected', 'waiting')).upper()} for active profile")

    parameter_pending = False
    parameter_blocked = False
    if parameter_snapshot_status is not None:
        normalized_parameter_status = str(parameter_snapshot_status).strip().lower()
        if normalized_parameter_status == "pending":
            parameter_pending = True
            reasons.append("PARAMETER SNAPSHOT PENDING")
        elif normalized_parameter_status != "available":
            parameter_blocked = True
            reasons.append(f"parameter snapshot {str(parameter_snapshot_status).upper()}")

    unique_reasons = tuple(dict.fromkeys(reasons))
    start_allowed = not parameter_blocked and not any(
        reason != "PARAMETER SNAPSHOT PENDING" for reason in unique_reasons
    )
    formal_eligible = start_allowed and not parameter_pending
    return RecordingReadiness(start_allowed, formal_eligible, unique_reasons)


def evaluate_recording_outcome(artifact_integrity, formal_eligible, eligibility_events=()):
    """Keep artifact completion separate from formal research eligibility."""
    events = tuple(eligibility_events or ())
    if not artifact_integrity:
        return {
            "recording_state": "error",
            "artifact_integrity": False,
            "formal_experiment_eligible": False,
        }
    formal = bool(formal_eligible) and not events
    return {
        "recording_state": "complete",
        "artifact_integrity": True,
        "formal_experiment_eligible": formal,
    }


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
    "RecordingReadiness", "RecordingReadinessModel", "PROFILE_NOT_CONFIGURED", "PROFILE_ACTIVE", "PROFILE_UNKNOWN_INVALID", "profile_contract_state", "disparity_difference_px", "latest_unsynchronized_disparity_difference_px", "intersection_angle_deg", "corrected_measurement_state", "state_display_value", "evaluate_recording_readiness", "evaluate_recording_outcome",
    "source_valid", "source_state", "source_state_from_monotonic_ns", "stale_display", "fallback_transition", "transition_events", "synchronized_comparison_errors",
    "plot_series_for_source", "should_append_plot", "stale_marker_action", "parse_px4_status", "px4_contract_state", "px4_overall_status", "mission_phase_display", "status_field", "format_status_vector", "format_receive_age",
]
