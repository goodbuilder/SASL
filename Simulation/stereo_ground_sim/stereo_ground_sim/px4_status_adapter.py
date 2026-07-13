"""Read-only PX4 telemetry adapter for the Simulation Operator Panel.

The adapter listens to the PX4 GCS MAVLink stream and publishes a small JSON
status message.  It never sends arm, mode, setpoint, takeoff, approach, or
landing commands.  The existing mission runner keeps ownership of that link
on the separate offboard port.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from typing import Any

PX4_STATUS_TOPIC = "/simulation/px4/status"
# PX4's posix startup script listens on 18570 (+ instance) for GCS clients and
# sends GCS telemetry to remote UDP port 14550.  QGroundControl normally owns
# 14550, while 14540 is the offboard control destination used by
# px4_auto_mission.py.  Use an outbound client socket to PX4's actual GCS
# listener so the read-only adapter does not bind or steal QGC's port.
DEFAULT_CONNECT = "udpout:127.0.0.1:18570"
PX4_FLIGHT_AUTHORITY = "px4_sitl"
FLIGHT_CONTRACT_SOURCE = "stereo_distance_node_parameters"
FLIGHT_CONTRACT_PARAMETER_NAMES = (
    "flight_authority",
    "fly_target_in_gazebo",
    "mission_profile",
    "actual_distance_source",
)
FLIGHT_CONTRACT_EXPECTED = {
    "flight_authority": "px4_sitl",
    "fly_target_in_gazebo": False,
    "mission_profile": "manual",
    "actual_distance_source": "gazebo",
}
FLIGHT_CONTRACT_MAX_AGE_S = 3.0
FLIGHT_CONTRACT_REQUEST_TIMEOUT_S = 2.0
HEARTBEAT_STALE_TIMEOUT_S = 3.0
LOCAL_POSITION_STALE_TIMEOUT_S = 3.0
MISSION_PHASES = (
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

# PX4 encodes the main and sub modes in the high bytes of HEARTBEAT.custom_mode.
# Keep this decoding local to the read-only adapter; it is not a flight-mode
# command path and does not duplicate the mission runner's control logic.
PX4_MAIN_MODE_NAMES = {
    1: "MANUAL",
    2: "ALTCTL",
    3: "POSCTL",
    4: "AUTO",
    5: "ACRO",
    6: "OFFBOARD",
    7: "STABILIZED",
    8: "RATTITUDE",
    9: "SIMPLE",
}
PX4_AUTO_SUB_MODE_NAMES = {
    1: "AUTO_READY",
    2: "AUTO_TAKEOFF",
    3: "AUTO_LOITER",
    4: "AUTO_MISSION",
    5: "AUTO_RTL",
    6: "AUTO_LAND",
    7: "AUTO_RTGS",
    8: "AUTO_FOLLOW_TARGET",
    9: "AUTO_PRECLAND",
}


def finite_number(value: Any) -> float | None:
    if isinstance(value, (bool, str)):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _finite_or_none(value: Any) -> float | None:
    return finite_number(value)


def _vector_or_none(values: Any, names: tuple[str, ...]) -> dict[str, float] | None:
    if values is None:
        return None
    result = {}
    for name in names:
        value = _finite_or_none(values.get(name) if isinstance(values, dict) else None)
        if value is None:
            return None
        result[name] = value
    return result


def _unverified_flight_contract(error: str) -> dict[str, Any]:
    """Return a contract state that cannot be mistaken for a verified one."""

    return {
        "verified": False,
        "source": FLIGHT_CONTRACT_SOURCE,
        "flight_authority": None,
        "fly_target_in_gazebo": None,
        "mission_profile": None,
        "actual_distance_source": None,
        "checked_monotonic_s": 0.0,
        "error": str(error),
    }


def _parameter_value(value: Any) -> Any:
    """Decode rclpy Parameter or rcl_interfaces ParameterValue objects."""

    if hasattr(value, "value"):
        return value.value
    parameter_type = getattr(value, "type", None)
    if parameter_type == 1:  # PARAMETER_BOOL
        return bool(getattr(value, "bool_value", False))
    if parameter_type == 2:  # PARAMETER_INTEGER
        return int(getattr(value, "integer_value", 0))
    if parameter_type == 3:  # PARAMETER_DOUBLE
        return float(getattr(value, "double_value", 0.0))
    if parameter_type == 4:  # PARAMETER_STRING
        return str(getattr(value, "string_value", ""))
    return value


def flight_contract_from_parameters(
    parameters: dict[str, Any] | None,
    *,
    checked_monotonic_s: float | None,
    now_monotonic_s: float,
    error: str | None = None,
    max_age_s: float = FLIGHT_CONTRACT_MAX_AGE_S,
) -> dict[str, Any]:
    """Validate the remote stereo node's live parameters.

    The adapter deliberately has no fallback to its own PX4-only constants.
    A missing/failed/stale remote lookup is always unverified.
    """

    result = _unverified_flight_contract(error or "stereo_distance_node parameters unavailable")
    if not isinstance(parameters, dict):
        return result
    checked = finite_number(checked_monotonic_s)
    now = finite_number(now_monotonic_s)
    if checked is None or now is None:
        result["error"] = "stereo_distance_node parameter check has no valid receive time"
        return result
    result["checked_monotonic_s"] = checked
    if now < checked or now - checked > max_age_s:
        result["error"] = "stereo_distance_node parameter check is stale"
        for name in FLIGHT_CONTRACT_PARAMETER_NAMES:
            if name in parameters:
                result[name] = _parameter_value(parameters[name])
        return result

    violations = []
    for name, expected in FLIGHT_CONTRACT_EXPECTED.items():
        if name not in parameters:
            violations.append(f"missing {name}")
            continue
        actual = _parameter_value(parameters[name])
        result[name] = actual
        if isinstance(expected, bool):
            matches = isinstance(actual, bool) and actual is expected
        else:
            matches = isinstance(actual, str) and actual == expected
        if not matches:
            violations.append(f"{name}={actual!r} (expected {expected!r})")
    if violations:
        result["error"] = "PX4-only flight-authority contract violation: " + "; ".join(violations)
        return result
    result["verified"] = True
    result["error"] = None
    return result


def mission_phase_provenance(
    phase: str,
    *,
    connected: bool,
    local_ned_m: dict[str, float] | None,
    velocity_ned_mps: dict[str, float] | None,
    authoritative_phase: str | None = None,
) -> tuple[str, str, str]:
    """Return display phase, provenance, and confidence without inventing authority."""

    if authoritative_phase in MISSION_PHASES and authoritative_phase not in {"UNAVAILABLE", "WAITING"}:
        return authoritative_phase, "authoritative_mission_event", "high"
    if not connected or phase in {"UNAVAILABLE", "WAITING"}:
        return "UNAVAILABLE", "unavailable", "none"
    has_position = isinstance(local_ned_m, dict) and all(
        finite_number(local_ned_m.get(name)) is not None for name in ("north", "east", "down")
    )
    has_velocity = isinstance(velocity_ned_mps, dict) and all(
        finite_number(velocity_ned_mps.get(name)) is not None for name in ("north", "east", "down")
    )
    if phase in {"TAKEOFF", "APPROACH", "HOVER_2", "LANDING"} and has_position and has_velocity:
        return phase, "inferred_mavlink_state", "medium"
    return phase, "inferred_mavlink_state", "low"


def px4_connection_snapshot(
    *,
    master_available: bool,
    last_heartbeat_monotonic_s: float | None,
    now_monotonic_s: float,
    connection_error: str | None = None,
    stale_timeout_s: float = HEARTBEAT_STALE_TIMEOUT_S,
) -> tuple[bool, str, str | None]:
    """Classify adapter transport state from heartbeat freshness and errors."""

    if not master_available:
        return False, "ERROR", connection_error or "MAVLink connection could not be opened"
    if connection_error:
        return False, "ERROR", connection_error
    heartbeat = finite_number(last_heartbeat_monotonic_s)
    now = finite_number(now_monotonic_s)
    if heartbeat is None or now is None or now < heartbeat or now - heartbeat > stale_timeout_s:
        return False, "DISCONNECTED", "PX4 heartbeat stale"
    return True, "OK", None


def local_position_snapshot(
    *,
    last_receive_monotonic_s: float | None,
    now_monotonic_s: float,
    stale_timeout_s: float = LOCAL_POSITION_STALE_TIMEOUT_S,
) -> tuple[str, bool]:
    """Return source-local LOCAL_POSITION_NED status and freshness."""

    received = finite_number(last_receive_monotonic_s)
    now = finite_number(now_monotonic_s)
    if received is None or now is None or now < received:
        return ("unavailable", False) if received is None else ("stale", False)
    if now - received > stale_timeout_s:
        return "stale", False
    return "valid", True


def decode_px4_custom_mode(custom_mode: Any) -> str | None:
    """Decode PX4 HEARTBEAT.custom_mode into a display-only mode label."""

    value = finite_number(custom_mode)
    if value is None:
        return None
    encoded = int(value)
    main_mode = (encoded >> 16) & 0xFF
    sub_mode = (encoded >> 24) & 0xFF
    main_name = PX4_MAIN_MODE_NAMES.get(main_mode)
    if main_name is None:
        return None
    if main_name == "AUTO":
        return PX4_AUTO_SUB_MODE_NAMES.get(sub_mode, main_name)
    return main_name


def navigation_mode_from_heartbeat(message: Any, mavlink_module: Any = None) -> str | None:
    """Return a useful PX4 mode from HEARTBEAT without trusting UNKNOWN."""

    decoded = decode_px4_custom_mode(getattr(message, "custom_mode", None))
    if decoded is not None:
        return decoded
    if mavlink_module is not None:
        try:
            mode = mavlink_module.mode_string_v10(message)
        except Exception:
            mode = None
        if mode and str(mode).upper() not in {"UNKNOWN", "UNAVAILABLE"}:
            return str(mode)
    return None


def infer_mission_phase(
    *,
    connected: bool,
    armed: bool,
    navigation_mode: str | None,
    local_ned_m: dict[str, float] | None,
    velocity_ned_mps: dict[str, float] | None,
    actual_distance_m: float | None = None,
    target_distance_m: float | None = None,
    previously_armed: bool = False,
) -> str:
    """Infer a display phase only from observed PX4 telemetry.

    This is a presentation classification, not a flight controller.  When a
    mission phase is unavailable from telemetry the adapter chooses a broad
    state rather than fabricating a command-runner phase.
    """

    if not connected:
        return "WAITING"
    mode = (navigation_mode or "").upper()
    if not armed:
        return "COMPLETE" if previously_armed else ("PRESTREAM" if mode == "OFFBOARD" else "CONNECTED")
    if mode in {"LAND", "AUTO_LAND"}:
        return "LANDING"
    down = None if local_ned_m is None else local_ned_m.get("down")
    vertical_speed = None if velocity_ned_mps is None else velocity_ned_mps.get("down")
    if down is not None and (
        abs(down) < 1.0
        or (down < -1.0 and vertical_speed is not None and vertical_speed < -0.5)
    ):
        return "TAKEOFF"
    speed = 0.0
    if velocity_ned_mps is not None:
        speed = math.sqrt(sum(velocity_ned_mps[name] ** 2 for name in ("north", "east", "down")))
    if (
        actual_distance_m is not None
        and target_distance_m is not None
        and actual_distance_m <= target_distance_m + 1.0
    ):
        return "HOVER_2"
    if speed > 0.5:
        return "APPROACH"
    return "HOVER_1" if mode == "OFFBOARD" else "OFFBOARD"


def build_status_payload(
    *,
    connected: bool,
    armed: bool,
    navigation_mode: str | None,
    failsafe: bool,
    mission_phase: str,
    local_ned_m: dict[str, float] | None,
    velocity_ned_mps: dict[str, float] | None,
    target_distance_m: float | None,
    actual_distance_m: float | None,
    sample_timestamp_us: int | None,
    receive_monotonic_s: float,
    status: str,
    error: str | None = None,
    flight_contract: dict[str, Any] | None = None,
    mission_phase_source: str | None = None,
    mission_phase_confidence: str | None = None,
    local_position_receive_monotonic_s: float | None = None,
    local_position_status: str | None = None,
    connection_source: str | None = None,
    endpoint_coexistence: str = "UNVERIFIED",
) -> dict[str, Any]:
    """Build the finite, additive JSON contract consumed by the UI."""

    if mission_phase not in MISSION_PHASES:
        mission_phase = "UNAVAILABLE"
    contract = flight_contract if isinstance(flight_contract, dict) else _unverified_flight_contract(
        "flight contract has not been verified"
    )
    if mission_phase_source not in {"authoritative_mission_event", "inferred_mavlink_state", "unavailable"}:
        mission_phase_source = "unavailable" if mission_phase == "UNAVAILABLE" else "inferred_mavlink_state"
    if mission_phase_confidence not in {"high", "medium", "low", "none"}:
        mission_phase_confidence = "none" if mission_phase_source == "unavailable" else "low"
    receive_time = finite_number(receive_monotonic_s)
    if receive_time is None:
        receive_time = 0.0
    local_receive_time = finite_number(local_position_receive_monotonic_s)
    if local_position_status not in {"valid", "stale", "unavailable"}:
        local_position_status = "valid" if local_receive_time is not None else "unavailable"
    return {
        "schema_version": 1,
        "flight_authority": contract.get("flight_authority"),
        "gazebo_motion_enabled": contract.get("fly_target_in_gazebo"),
        "mission_profile": contract.get("mission_profile"),
        "actual_distance_source": contract.get("actual_distance_source"),
        "flight_contract": contract,
        "px4_connected": bool(connected),
        "armed": bool(armed),
        "navigation_mode": str(navigation_mode or "unavailable") if connected else "unavailable",
        "failsafe": bool(failsafe),
        "mission_phase": mission_phase,
        "mission_phase_source": mission_phase_source,
        "mission_phase_confidence": mission_phase_confidence,
        "local_ned_m": _vector_or_none(local_ned_m, ("north", "east", "down")),
        "velocity_ned_mps": _vector_or_none(velocity_ned_mps, ("north", "east", "down")),
        "local_position_status": local_position_status,
        "local_position_receive_monotonic_s": local_receive_time,
        "target_distance_m": _finite_or_none(target_distance_m),
        "actual_distance_m": _finite_or_none(actual_distance_m),
        "sample_timestamp_us": None if sample_timestamp_us is None else int(sample_timestamp_us),
        "receive_monotonic_s": receive_time,
        "status_connection_source": connection_source or "MAVLink unavailable",
        "endpoint_coexistence": str(endpoint_coexistence),
        "status": str(status),
        "error": None if error is None else str(error),
    }


try:  # Keep pure helpers importable on hosts without ROS or pymavlink.
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import Float32, String
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - exercised on non-runtime hosts.
    rclpy = None
    Node = object  # type: ignore[assignment,misc]
    GetParameters = None
    Float32 = String = None
    mavutil = None


class Px4StatusAdapter(Node):
    """Read PX4 GCS telemetry and publish status without commanding PX4."""

    def __init__(self, connect: str | None = None, publish_hz: float = 5.0) -> None:
        super().__init__("px4_status_adapter")
        self.declare_parameter("connect", connect or os.environ.get("PX4_STATUS_CONNECT", DEFAULT_CONNECT))
        self.declare_parameter("publish_hz", publish_hz)
        self.connect = str(self.get_parameter("connect").value)
        self.publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.publisher = self.create_publisher(String, PX4_STATUS_TOPIC, 10)
        self.actual_distance_m: float | None = None
        self.target_distance_m: float | None = None
        self.create_subscription(Float32, "/ground_truth", self._actual_distance_cb, 10)
        self.create_subscription(Float32, "/target_distance", self._target_distance_cb, 10)
        self.master = None
        self.last_heartbeat_monotonic = None
        self.last_receive_monotonic = time.monotonic()
        self.last_gcs_heartbeat_sent_monotonic = None
        self.last_local_position_monotonic = None
        self.armed = False
        self.previously_armed = False
        self.navigation_mode = "unavailable"
        self.failsafe = False
        self.local_ned_m = None
        self.velocity_ned_mps = None
        self.sample_timestamp_us = None
        self.connection_error = None
        self.flight_contract = _unverified_flight_contract(
            "stereo_distance_node parameter service unavailable"
        )
        self._flight_contract_client = (
            self.create_client(GetParameters, "/stereo_distance_node/get_parameters")
            if GetParameters is not None
            else None
        )
        self._flight_contract_future = None
        self._flight_contract_request_monotonic = None
        self._flight_contract_next_request_monotonic = 0.0
        if mavutil is not None:
            try:
                self.master = mavutil.mavlink_connection(self.connect, autoreconnect=True, source_system=255)
            except Exception as exc:  # pragma: no cover - depends on runtime socket.
                self.connection_error = f"MAVLink connection unavailable: {exc}"
        else:
            self.connection_error = "pymavlink unavailable"
        self.create_timer(1.0 / self.publish_hz, self._poll)

    @staticmethod
    def _parameter_values_from_result(result: Any) -> dict[str, Any]:
        values = getattr(result, "values", result)
        if not isinstance(values, (list, tuple)):
            raise RuntimeError("parameter service returned no values")
        if len(values) != len(FLIGHT_CONTRACT_PARAMETER_NAMES):
            raise RuntimeError(
                "parameter service returned an unexpected value count: "
                f"{len(values)}"
            )
        return {
            name: _parameter_value(value)
            for name, value in zip(FLIGHT_CONTRACT_PARAMETER_NAMES, values)
        }

    def _poll_flight_contract(self, now: float) -> None:
        client = self._flight_contract_client
        if client is None:
            self.flight_contract = _unverified_flight_contract(
                "ROS GetParameters service client unavailable"
            )
            return

        future = self._flight_contract_future
        if future is not None:
            if future.done():
                try:
                    values = self._parameter_values_from_result(future.result())
                    self.flight_contract = flight_contract_from_parameters(
                        values,
                        checked_monotonic_s=now,
                        now_monotonic_s=now,
                    )
                except Exception as exc:
                    self.flight_contract = _unverified_flight_contract(
                        f"stereo_distance_node parameter lookup failed: {exc}"
                    )
                self._flight_contract_future = None
                self._flight_contract_request_monotonic = None
                self._flight_contract_next_request_monotonic = now + 1.0
            elif (
                self._flight_contract_request_monotonic is not None
                and now - self._flight_contract_request_monotonic > FLIGHT_CONTRACT_REQUEST_TIMEOUT_S
            ):
                self.flight_contract = _unverified_flight_contract(
                    "stereo_distance_node parameter lookup timed out"
                )
                self._flight_contract_future = None
                self._flight_contract_request_monotonic = None
                self._flight_contract_next_request_monotonic = now + 1.0

        if (
            self._flight_contract_future is None
            and now >= self._flight_contract_next_request_monotonic
        ):
            try:
                if not client.service_is_ready():
                    self.flight_contract = _unverified_flight_contract(
                        "stereo_distance_node parameter service unavailable"
                    )
                    self._flight_contract_next_request_monotonic = now + 1.0
                    return
                request = GetParameters.Request()
                request.names = list(FLIGHT_CONTRACT_PARAMETER_NAMES)
                self._flight_contract_future = client.call_async(request)
                self._flight_contract_request_monotonic = now
            except Exception as exc:
                self.flight_contract = _unverified_flight_contract(
                    f"stereo_distance_node parameter service unavailable: {exc}"
                )
                self._flight_contract_next_request_monotonic = now + 1.0

    def _actual_distance_cb(self, msg: Any) -> None:
        self.actual_distance_m = finite_number(getattr(msg, "data", None))

    def _target_distance_cb(self, msg: Any) -> None:
        self.target_distance_m = finite_number(getattr(msg, "data", None))

    def _handle_message(self, message: Any) -> None:
        message_type = message.get_type()
        now = time.monotonic()
        self.last_receive_monotonic = now
        self.connection_error = None
        if message_type == "HEARTBEAT":
            autopilot = int(getattr(message, "autopilot", -1))
            if mavutil is not None and autopilot not in (mavutil.mavlink.MAV_AUTOPILOT_PX4, -1):
                return
            self.last_heartbeat_monotonic = now
            base_mode = int(getattr(message, "base_mode", 0))
            self.armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.previously_armed = self.previously_armed or self.armed
            self.navigation_mode = navigation_mode_from_heartbeat(message, mavutil)
            state = int(getattr(message, "system_status", 0))
            self.failsafe = state in (
                mavutil.mavlink.MAV_STATE_CRITICAL,
                mavutil.mavlink.MAV_STATE_EMERGENCY,
            )
        elif message_type == "LOCAL_POSITION_NED":
            self.last_local_position_monotonic = now
            self.local_ned_m = {
                "north": float(message.x),
                "east": float(message.y),
                "down": float(message.z),
            }
            self.velocity_ned_mps = {
                "north": float(message.vx),
                "east": float(message.vy),
                "down": float(message.vz),
            }
            boot_ms = finite_number(getattr(message, "time_boot_ms", None))
            self.sample_timestamp_us = None if boot_ms is None else int(boot_ms * 1000.0)

    def _send_gcs_heartbeat(self, now: float) -> None:
        """Register this read-only UDP client with PX4's GCS MAVLink link."""

        if self.master is None or mavutil is None:
            return
        if (
            self.last_gcs_heartbeat_sent_monotonic is not None
            and now - self.last_gcs_heartbeat_sent_monotonic < 1.0
        ):
            return
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.last_gcs_heartbeat_sent_monotonic = now
        except Exception as exc:  # pragma: no cover - depends on runtime socket.
            self.connection_error = f"MAVLink heartbeat send error: {exc}"

    def _poll(self) -> None:
        now = time.monotonic()
        self._poll_flight_contract(now)
        self._send_gcs_heartbeat(now)
        if self.master is not None:
            try:
                for _ in range(100):
                    message = self.master.recv_match(blocking=False)
                    if message is None:
                        break
                    self._handle_message(message)
            except Exception as exc:  # pragma: no cover - depends on socket state.
                self.connection_error = f"MAVLink receive error: {exc}"
        now = time.monotonic()
        connected, adapter_status, adapter_error = px4_connection_snapshot(
            master_available=self.master is not None,
            last_heartbeat_monotonic_s=self.last_heartbeat_monotonic,
            now_monotonic_s=now,
            connection_error=self.connection_error,
        )
        if not connected:
            self.armed = False
        local_position_status, local_position_fresh = local_position_snapshot(
            last_receive_monotonic_s=self.last_local_position_monotonic,
            now_monotonic_s=now,
        )
        local_ned = self.local_ned_m if connected and local_position_fresh else None
        velocity_ned = self.velocity_ned_mps if connected and local_position_fresh else None
        phase = infer_mission_phase(
            connected=connected,
            armed=self.armed,
            navigation_mode=self.navigation_mode,
            local_ned_m=local_ned,
            velocity_ned_mps=velocity_ned,
            actual_distance_m=self.actual_distance_m,
            target_distance_m=self.target_distance_m,
            previously_armed=self.previously_armed,
        )
        phase, phase_source, phase_confidence = mission_phase_provenance(
            phase,
            connected=connected,
            local_ned_m=local_ned,
            velocity_ned_mps=velocity_ned,
        )
        if not connected:
            self.armed = False
            local_ned = None
            velocity_ned = None
            local_position_status = "stale" if self.last_local_position_monotonic is not None else "unavailable"
        payload = build_status_payload(
            connected=connected,
            armed=self.armed,
            navigation_mode=self.navigation_mode,
            failsafe=self.failsafe,
            mission_phase=phase,
            local_ned_m=local_ned,
            velocity_ned_mps=velocity_ned,
            target_distance_m=self.target_distance_m,
            actual_distance_m=self.actual_distance_m,
            sample_timestamp_us=self.sample_timestamp_us,
            receive_monotonic_s=now,
            status=adapter_status,
            error=adapter_error,
            flight_contract=flight_contract_from_parameters(
                {
                    name: self.flight_contract.get(name)
                    for name in FLIGHT_CONTRACT_PARAMETER_NAMES
                    if name in self.flight_contract
                },
                checked_monotonic_s=self.flight_contract.get("checked_monotonic_s"),
                now_monotonic_s=now,
                error=self.flight_contract.get("error"),
            ),
            mission_phase_source=phase_source,
            mission_phase_confidence=phase_confidence,
            local_position_receive_monotonic_s=(
                self.last_local_position_monotonic if local_position_fresh else None
            ),
            local_position_status=local_position_status,
            connection_source=f"MAVLink {self.connect}",
            endpoint_coexistence="UNVERIFIED",
        )
        self.publisher.publish(String(data=json.dumps(payload, allow_nan=False, sort_keys=True)))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish read-only PX4 telemetry for the Simulation UI.")
    parser.add_argument("--connect", default=os.environ.get("PX4_STATUS_CONNECT", DEFAULT_CONNECT))
    parser.add_argument("--publish-hz", type=float, default=5.0)
    return parser


def main(args: list[str] | None = None) -> int:
    if rclpy is None:  # pragma: no cover
        raise RuntimeError("ROS 2 and pymavlink are required for px4_status_adapter")
    parsed = build_parser().parse_args(args)
    rclpy.init()
    node = Px4StatusAdapter(connect=parsed.connect, publish_hz=parsed.publish_hz)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, getattr(rclpy.executors, "ExternalShutdownException", KeyboardInterrupt)):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


__all__ = [
    "DEFAULT_CONNECT",
    "FLIGHT_CONTRACT_EXPECTED",
    "FLIGHT_CONTRACT_PARAMETER_NAMES",
    "FLIGHT_CONTRACT_SOURCE",
    "MISSION_PHASES",
    "PX4_FLIGHT_AUTHORITY",
    "PX4_STATUS_TOPIC",
    "build_status_payload",
    "finite_number",
    "flight_contract_from_parameters",
    "infer_mission_phase",
    "local_position_snapshot",
    "mission_phase_provenance",
    "px4_connection_snapshot",
    "Px4StatusAdapter",
    "main",
]


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
