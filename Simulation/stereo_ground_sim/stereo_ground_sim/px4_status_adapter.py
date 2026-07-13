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
DEFAULT_CONNECT = "udpin:0.0.0.0:14550"
PX4_FLIGHT_AUTHORITY = "px4_sitl"
MISSION_PHASES = (
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
) -> dict[str, Any]:
    """Build the finite, additive JSON contract consumed by the UI."""

    if mission_phase not in MISSION_PHASES:
        mission_phase = "ERROR"
    return {
        "schema_version": 1,
        "flight_authority": PX4_FLIGHT_AUTHORITY,
        "gazebo_motion_enabled": False,
        "px4_connected": bool(connected),
        "armed": bool(armed),
        "navigation_mode": str(navigation_mode or "unavailable"),
        "failsafe": bool(failsafe),
        "mission_phase": mission_phase,
        "local_ned_m": _vector_or_none(local_ned_m, ("north", "east", "down")),
        "velocity_ned_mps": _vector_or_none(velocity_ned_mps, ("north", "east", "down")),
        "target_distance_m": _finite_or_none(target_distance_m),
        "actual_distance_m": _finite_or_none(actual_distance_m),
        "sample_timestamp_us": None if sample_timestamp_us is None else int(sample_timestamp_us),
        "receive_monotonic_s": float(receive_monotonic_s),
        "status": str(status),
        "error": None if error is None else str(error),
    }


try:  # Keep pure helpers importable on hosts without ROS or pymavlink.
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, String
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - exercised on non-runtime hosts.
    rclpy = None
    Node = object  # type: ignore[assignment,misc]
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
        self.armed = False
        self.previously_armed = False
        self.navigation_mode = "unavailable"
        self.failsafe = False
        self.local_ned_m = None
        self.velocity_ned_mps = None
        self.sample_timestamp_us = None
        self.connection_error = None
        if mavutil is not None:
            try:
                self.master = mavutil.mavlink_connection(self.connect, autoreconnect=True, source_system=255)
            except Exception as exc:  # pragma: no cover - depends on runtime socket.
                self.connection_error = f"MAVLink connection unavailable: {exc}"
        else:
            self.connection_error = "pymavlink unavailable"
        self.create_timer(1.0 / self.publish_hz, self._poll)

    def _actual_distance_cb(self, msg: Any) -> None:
        self.actual_distance_m = finite_number(getattr(msg, "data", None))

    def _target_distance_cb(self, msg: Any) -> None:
        self.target_distance_m = finite_number(getattr(msg, "data", None))

    def _handle_message(self, message: Any) -> None:
        message_type = message.get_type()
        now = time.monotonic()
        self.last_receive_monotonic = now
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

    def _poll(self) -> None:
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
        connected = self.last_heartbeat_monotonic is not None and now - self.last_heartbeat_monotonic <= 3.0
        if not connected:
            self.armed = False
        phase = infer_mission_phase(
            connected=connected,
            armed=self.armed,
            navigation_mode=self.navigation_mode,
            local_ned_m=self.local_ned_m,
            velocity_ned_mps=self.velocity_ned_mps,
            actual_distance_m=self.actual_distance_m,
            target_distance_m=self.target_distance_m,
            previously_armed=self.previously_armed,
        )
        payload = build_status_payload(
            connected=connected,
            armed=self.armed,
            navigation_mode=self.navigation_mode,
            failsafe=self.failsafe,
            mission_phase=phase,
            local_ned_m=self.local_ned_m,
            velocity_ned_mps=self.velocity_ned_mps,
            target_distance_m=self.target_distance_m,
            actual_distance_m=self.actual_distance_m,
            sample_timestamp_us=self.sample_timestamp_us,
            receive_monotonic_s=now,
            status="ok" if connected else "unavailable",
            error=None if connected else (self.connection_error or "PX4 heartbeat unavailable"),
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
    "MISSION_PHASES",
    "PX4_FLIGHT_AUTHORITY",
    "PX4_STATUS_TOPIC",
    "build_status_payload",
    "finite_number",
    "infer_mission_phase",
    "Px4StatusAdapter",
    "main",
]


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
