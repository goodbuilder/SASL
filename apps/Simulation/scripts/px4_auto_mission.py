#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil


def positive_float(value):
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def bounded_ratio(value):
    parsed = float(value)
    if not 0.0 <= parsed < 0.5:
        raise argparse.ArgumentTypeError("must be in the range [0, 0.5)")
    return parsed


def nonnegative_float(value):
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def triangle_wave(elapsed_s, period_s, amplitude_m):
    """Symmetric triangle wave: +amplitude at elapsed=0, -amplitude at the half period."""

    if period_s <= 0 or amplitude_m <= 0:
        return 0.0
    phase = (elapsed_s % period_s) / period_s
    return amplitude_m * (4.0 * abs(phase - 0.5) - 1.0)


def bbox_inside_safe_fov(bbox, image_width_px, image_height_px, margin_ratio):
    if bbox is None or len(bbox) != 4:
        return False
    try:
        x1, y1, x2, y2 = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        return False
    margin_x = float(image_width_px) * float(margin_ratio)
    margin_y = float(image_height_px) * float(margin_ratio)
    return (
        x2 > x1
        and y2 > y1
        and x1 >= margin_x
        and y1 >= margin_y
        and x2 <= float(image_width_px) - margin_x
        and y2 <= float(image_height_px) - margin_y
    )


def bbox_fills_safe_margin(bbox, image_width_px, image_height_px, margin_ratio):
    """True once all four bbox edges have reached their margin.

    Unlike ``bbox_inside_safe_fov`` (which fails once an edge crosses past the
    margin), this is the approach-side condition: the target has grown to
    exactly span the safe box between the margins on every side, without yet
    violating any of them.
    """
    if bbox is None or len(bbox) != 4:
        return False
    try:
        x1, y1, x2, y2 = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        return False
    margin_x = float(image_width_px) * float(margin_ratio)
    margin_y = float(image_height_px) * float(margin_ratio)
    return (
        x1 <= margin_x
        and y1 <= margin_y
        and x2 >= float(image_width_px) - margin_x
        and y2 >= float(image_height_px) - margin_y
    )


def parse_xy_alt(entry):
    parts = [part.strip() for part in entry.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("waypoint must be north,east,altitude_agl_m")
    north, east, altitude = (float(part) for part in parts)
    if altitude < 0:
        raise argparse.ArgumentTypeError("altitude_agl_m must be zero or greater")
    return north, east, -altitude


def parse_waypoint_list(value):
    waypoints = []
    for entry in value.split(";"):
        entry = entry.strip()
        if entry:
            waypoints.append(parse_xy_alt(entry))
    return waypoints


def read_path_file(path):
    waypoints = []
    with Path(path).open(newline="") as handle:
        sample = handle.read(2048)
        handle.seek(0)
        has_header = csv.Sniffer().has_header(sample) if sample.strip() else False
        if has_header:
            reader = csv.DictReader(
                row for row in handle if row.strip() and not row.lstrip().startswith("#")
            )
            for row in reader:
                north = row.get("north") or row.get("x") or row.get("north_m")
                east = row.get("east") or row.get("y") or row.get("east_m")
                altitude = (
                    row.get("altitude")
                    or row.get("alt")
                    or row.get("altitude_m")
                    or row.get("altitude_agl_m")
                )
                if north is None or east is None or altitude is None:
                    raise ValueError(
                        "CSV header must include north/east/altitude columns"
                    )
                waypoints.append(parse_xy_alt(f"{north},{east},{altitude}"))
        else:
            reader = csv.reader(
                row for row in handle if row.strip() and not row.lstrip().startswith("#")
            )
            for row in reader:
                waypoints.append(parse_xy_alt(",".join(row)))
    return waypoints


def build_parser():
    parser = argparse.ArgumentParser(
        description="Run a PX4 offboard takeoff, body-forward approach, and landing mission."
    )
    parser.add_argument(
        "--connect",
        default="udpin:0.0.0.0:14540",
        help="pymavlink connection string. Default: udpin:0.0.0.0:14540",
    )
    parser.add_argument("--altitude-m", type=positive_float, default=10.0)
    parser.add_argument(
        "--mpc-z-vel-max-up-mps",
        type=positive_float,
        default=None,
        help="Override PX4 MPC_Z_VEL_MAX_UP (climb speed limit, default PX4 value: 3 m/s) "
        "before takeoff. Unset leaves PX4's configured value alone.",
    )
    parser.add_argument(
        "--mpc-z-vel-max-dn-mps",
        type=positive_float,
        default=None,
        help="Override PX4 MPC_Z_VEL_MAX_DN (descent speed limit, default PX4 value: 1.5 m/s) "
        "before takeoff. Unset leaves PX4's configured value alone.",
    )
    parser.add_argument("--distance-m", type=positive_float, default=1900.0)
    parser.add_argument("--target-distance-m", type=positive_float, default=200.0)
    parser.add_argument("--speed-mps", type=positive_float, default=15.0)
    parser.add_argument("--approach-timeout-s", type=positive_float, default=600.0)
    parser.add_argument("--takeoff-timeout-s", type=positive_float, default=60.0)
    parser.add_argument("--first-hover-s", type=float, default=5.0)
    parser.add_argument("--final-hover-s", type=float, default=5.0)
    parser.add_argument("--land-timeout-s", type=positive_float, default=90.0)
    parser.add_argument("--setpoint-hz", type=positive_float, default=20.0)
    parser.add_argument("--heartbeat-timeout-s", type=positive_float, default=30.0)
    parser.add_argument(
        "--fov-guard",
        action="store_true",
        help="Require both full detection bounding boxes to stay inside the safe FOV area.",
    )
    parser.add_argument(
        "--bbox-margin-ratio",
        type=bounded_ratio,
        default=float(os.environ.get("AUTO_FOV_MARGIN_RATIO", "0.05")),
        help="Inset each image edge by this ratio before allowing forward motion.",
    )
    parser.add_argument(
        "--fov-detection-timeout-s",
        type=positive_float,
        default=float(os.environ.get("AUTO_FOV_DETECTION_TIMEOUT_S", "0.5")),
        help="Maximum age of the paired left/right detections.",
    )
    parser.add_argument(
        "--check-connection",
        action="store_true",
        help="Connect to PX4 and print local position without arming or moving.",
    )
    parser.add_argument(
        "--lateral-bias-m",
        type=float,
        default=0.0,
        help="Constant lateral offset (m, along the approach's lateral axis) the "
        "approach controller holds the target away from image-center. Samples "
        "off-boresight viewing geometry; 0.0 keeps the existing centered approach.",
    )
    parser.add_argument(
        "--lateral-sweep-amplitude-m",
        type=nonnegative_float,
        default=0.0,
        help="Amplitude (m) of a continuous triangle-wave lateral sweep added on top "
        "of --lateral-bias-m during the approach, so one flight samples a range of "
        "off-boresight offsets instead of one fixed value. The safe-FOV guard still "
        "freezes all motion if a bbox nears the margin. 0.0 disables the sweep.",
    )
    parser.add_argument(
        "--lateral-sweep-period-s",
        type=positive_float,
        default=24.0,
        help="Period (s) of the lateral sweep triangle wave. Ignored when "
        "--lateral-sweep-amplitude-m is 0.",
    )
    parser.add_argument(
        "--waypoints-xy-alt",
        default="",
        help="Semicolon-separated north,east,altitude_agl_m list. When set, the "
        "mission flies this path (instead of the straight --altitude-m/--distance-m "
        "approach) and, with --fov-guard, stops to hover in place the instant both "
        "cameras' bboxes span the full safe box on every side, rather than "
        "flying every waypoint.",
    )
    parser.add_argument(
        "--waypoint",
        action="append",
        default=[],
        type=parse_xy_alt,
        metavar="NORTH,EAST,ALT",
        help="Add one north,east,altitude_agl_m waypoint. Repeatable.",
    )
    parser.add_argument(
        "--path-file",
        help="Read waypoints from a north,east,altitude_agl_m CSV/text file "
        "(e.g. the *_mission.csv written by generate_fov_constrained_waypoints.py).",
    )
    parser.add_argument(
        "--waypoint-acceptance-radius-m",
        type=positive_float,
        default=5.0,
        help="Distance (m) at which a waypoint counts as reached. Default: 5.",
    )
    return parser


class ActualDistanceMonitor:
    def __init__(self, topic="/ground_truth"):
        self.topic = topic
        self.value = None
        self._node = None
        self._thread = None
        self._executor = None
        self._rclpy = None
        self._owns_rclpy = False
        self._ready = False
        self._phase_publish_warning_logged = False
        self._stop = threading.Event()

    def start(self):
        try:
            import rclpy
            from std_msgs.msg import Float32, String
        except Exception as exc:
            print(f"Actual-distance ROS monitor unavailable: {exc}", flush=True)
            return

        self._rclpy = rclpy
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy = True
            self._node = rclpy.create_node("px4_auto_mission_distance_monitor")
            self._node.create_subscription(Float32, self.topic, self._callback, 10)
            self._phase_pub = self._node.create_publisher(
                String, "/stereo/auto_mission_phase", 10
            )
            self._phase_type = String
            self._ready = True
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
        except Exception as exc:
            print(f"Actual-distance ROS monitor unavailable: {exc}", flush=True)
            self._ready = False
            self._node = None

    def _callback(self, msg):
        self.value = float(msg.data)

    def publish_phase(self, phase):
        if self._node is None or not self._ready:
            return
        try:
            if self._rclpy is not None and not self._rclpy.ok():
                return
            self._phase_pub.publish(self._phase_type(data=str(phase)))
        except Exception as exc:
            if not self._phase_publish_warning_logged:
                print(
                    f"Auto mission phase topic unavailable; continuing mission: {exc}",
                    file=sys.stderr,
                    flush=True,
                )
                self._phase_publish_warning_logged = True

    def _spin(self):
        while not self._stop.is_set() and self._node is not None:
            try:
                if self._executor is not None:
                    self._executor.spin_once(timeout_sec=0.1)
                else:
                    self._rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                if not self._stop.is_set():
                    self._ready = False
                return

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._node is not None:
            try:
                if self._executor is not None:
                    self._executor.remove_node(self._node)
                self._node.destroy_node()
            except Exception:
                pass
        if self._rclpy is not None and self._owns_rclpy:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
            self._owns_rclpy = False

    @property
    def ready(self):
        return self._ready


class BboxFovMonitor:
    """Guard known bbox geometry without making detection a movement prerequisite."""

    def __init__(
        self,
        topic="/stereo/live_detections",
        image_width_px=1280,
        image_height_px=720,
        margin_ratio=0.05,
        timeout_s=0.5,
    ):
        self.topic = topic
        self.image_width_px = int(image_width_px)
        self.image_height_px = int(image_height_px)
        self.margin_ratio = float(margin_ratio)
        self.timeout_s = float(timeout_s)
        self._rclpy = None
        self._node = None
        self._thread = None
        self._executor = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._latest_time = None
        self.error = None

    def start(self):
        try:
            import rclpy
            from std_msgs.msg import Float32MultiArray
        except Exception as exc:
            self.error = f"ROS bbox monitor unavailable: {exc}"
            return

        self._rclpy = rclpy
        try:
            try:
                from rclpy.executors import SingleThreadedExecutor
            except Exception:
                SingleThreadedExecutor = None
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node("px4_auto_fov_bbox_guard")
            self._node.create_subscription(
                Float32MultiArray,
                self.topic,
                self._callback,
                10,
            )
            if SingleThreadedExecutor is not None:
                self._executor = SingleThreadedExecutor()
                self._executor.add_node(self._node)
        except Exception as exc:
            self.error = f"ROS bbox monitor failed to start: {exc}"
            return
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _callback(self, msg):
        values = list(getattr(msg, "data", []))
        if len(values) < 16:
            return
        parsed = []
        for offset in (0, 8):
            try:
                valid = float(values[offset]) > 0.5
                bbox = tuple(float(values[offset + index]) for index in range(1, 5))
            except (TypeError, ValueError):
                return
            if not valid or not all(math.isfinite(value) for value in bbox):
                parsed.append(None)
            else:
                parsed.append(bbox)
        with self._lock:
            self._latest = tuple(parsed)
            self._latest_time = time.monotonic()

    def _spin(self):
        while not self._stop.is_set() and self._node is not None:
            try:
                if self._executor is not None:
                    self._executor.spin_once(timeout_sec=0.1)
                else:
                    self._rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                if self._stop.is_set():
                    return

    def snapshot(self):
        with self._lock:
            latest = self._latest
            latest_time = self._latest_time
        if self.error:
            return True, f"FOV monitor unavailable; movement not gated ({self.error})"
        if latest is None or latest_time is None:
            return True, "no paired detections; movement not gated"
        age = time.monotonic() - latest_time
        if age > self.timeout_s:
            return True, f"detections stale ({age:.2f}s); movement not gated"
        if latest[0] is None or latest[1] is None:
            return True, "left/right full bbox pair unavailable; movement not gated"

        for side, bbox in zip(("left", "right"), latest):
            if not bbox_inside_safe_fov(
                bbox,
                self.image_width_px,
                self.image_height_px,
                self.margin_ratio,
            ):
                return False, f"{side} full bbox is outside the safe FOV margin"
        return True, "both full bboxes inside safe FOV margin"

    def filled_snapshot(self):
        """True once both cameras' bboxes span the full safe box on every side.

        Unlike ``snapshot()``, missing/stale/unpaired detections count as
        "not filled" here -- filling the margin is a positive claim that
        requires real, fresh, paired detections to back it up.
        """
        with self._lock:
            latest = self._latest
            latest_time = self._latest_time
        if self.error:
            return False, f"FOV monitor unavailable ({self.error})"
        if latest is None or latest_time is None:
            return False, "no paired detections; margin-fill undetermined"
        age = time.monotonic() - latest_time
        if age > self.timeout_s:
            return False, f"detections stale ({age:.2f}s); margin-fill undetermined"
        if latest[0] is None or latest[1] is None:
            return False, "left/right full bbox pair unavailable; margin-fill undetermined"

        for side, bbox in zip(("left", "right"), latest):
            if not bbox_fills_safe_margin(
                bbox, self.image_width_px, self.image_height_px, self.margin_ratio
            ):
                return False, f"{side} bbox has not yet reached all four margins"
        return True, "both left and right bboxes span the full safe box on every side"

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._node is not None:
            try:
                if self._executor is not None:
                    self._executor.remove_node(self._node)
                self._node.destroy_node()
            except Exception:
                pass


class GcsHeartbeat:
    def __init__(self, master, hz=1.0):
        self.master = master
        self.period_s = 1.0 / hz
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self._stop.wait(self.period_s)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


def now_ms(start_time):
    return int((time.monotonic() - start_time) * 1000) & 0xFFFFFFFF


def wait_heartbeat(master, timeout_s):
    print(f"Waiting for PX4 heartbeat on {master.address} ...", flush=True)
    deadline = time.monotonic() + timeout_s
    heartbeat = None
    while time.monotonic() < deadline:
        candidate = master.wait_heartbeat(timeout=1.0)
        if candidate is None:
            continue
        autopilot = getattr(candidate, "autopilot", None)
        system = int(getattr(candidate, "get_srcSystem", lambda: master.target_system)())
        if system > 0 and autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            heartbeat = candidate
            master.target_system = system
            master.target_component = int(
                getattr(candidate, "get_srcComponent", lambda: master.target_component)()
            )
            break
        print(
            "  ignoring non-PX4 heartbeat: "
            f"system={system}, autopilot={autopilot}",
            flush=True,
        )
    if heartbeat is None:
        raise RuntimeError(
            "PX4 heartbeat not received. Start ./run_operator_view.sh first, "
            "or try --connect udpout:127.0.0.1:14540."
        )
    print(
        f"Connected: system={master.target_system}, component={master.target_component}",
        flush=True,
    )


def wait_local_position(master, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if msg is not None:
            return msg
    raise RuntimeError("LOCAL_POSITION_NED was not received from PX4.")


def set_mode(master, mode_name):
    mapping = master.mode_mapping()
    if mode_name not in mapping:
        available = ", ".join(sorted(mapping.keys()))
        raise RuntimeError(f"PX4 mode {mode_name!r} is unavailable. Available: {available}")
    mode_id = mapping[mode_name]
    if isinstance(mode_id, tuple):
        master.set_mode_px4(*mode_id)
    else:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            int(mode_id),
        )
    print(f"Mode command sent: {mode_name}", flush=True)


def command_long(master, command, *params):
    padded = list(params)[:7] + [0.0] * (7 - len(params))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        *padded,
    )


def request_land(master):
    """Ask PX4 to land, including when the offboard mission is interrupted."""
    if "LAND" in master.mode_mapping():
        set_mode(master, "LAND")
    else:
        command_long(master, mavutil.mavlink.MAV_CMD_NAV_LAND)


def wait_command_ack(master, command, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack is None:
            continue
        if int(ack.command) == int(command):
            return ack
    return None


def set_param_float(master, name, value, attempts=4, retry_sleep_s=0.5, timeout_s=2.0):
    """Set a PX4 float parameter and confirm it via the echoed PARAM_VALUE."""
    if len(name) > 16:
        raise ValueError(f"PX4 param id {name!r} exceeds the 16-character MAVLink limit")
    last_seen = None
    for attempt in range(1, attempts + 1):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode("utf-8"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.param_id.rstrip("\x00") != name:
                continue
            last_seen = float(msg.param_value)
            if math.isclose(last_seen, float(value), rel_tol=1.0e-3, abs_tol=1.0e-3):
                print(f"Param {name} set to {last_seen:g}", flush=True)
                return last_seen
        if attempt < attempts:
            time.sleep(retry_sleep_s)
    seen_note = "" if last_seen is None else f" (last observed value: {last_seen!r})"
    raise RuntimeError(
        f"PX4 did not confirm {name}={value!r} after {attempts} attempts{seen_note}"
    )


def arm(master, attempts=4, retry_sleep_s=2.0):
    print("Arming ...", flush=True)
    last_result = None
    for attempt in range(1, attempts + 1):
        command_long(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
        ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack is not None:
            last_result = ack.result
            if ack.result in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                return
            print(f"Arm attempt {attempt}/{attempts} rejected by PX4: result={ack.result}", flush=True)
        else:
            print(f"Arm attempt {attempt}/{attempts} was not acknowledged by PX4.", flush=True)
        if attempt < attempts:
            time.sleep(retry_sleep_s)
    if last_result is not None:
        raise RuntimeError(f"Arm rejected by PX4 after {attempts} attempts: result={last_result}")
    raise RuntimeError(f"Arm command was not acknowledged by PX4 after {attempts} attempts.")


def send_position_setpoint(master, boot_time, x, y, z):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        now_ms(boot_time),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x,
        y,
        z,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_body_velocity_setpoint(master, boot_time, forward_mps, right_mps=0.0, down_mps=0.0):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        now_ms(boot_time),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0.0,
        0.0,
        0.0,
        forward_mps,
        right_mps,
        down_mps,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_local_velocity_setpoint(master, boot_time, north_mps=0.0, east_mps=0.0, down_mps=0.0):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        now_ms(boot_time),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0,
        0.0,
        0.0,
        north_mps,
        east_mps,
        down_mps,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def stream_for(duration_s, hz, callback):
    if duration_s <= 0:
        return
    period_s = 1.0 / hz
    end_time = time.monotonic() + duration_s
    while time.monotonic() < end_time:
        callback()
        time.sleep(period_s)


def stream_until_interrupted(hz, callback):
    """Stream a setpoint forever; Ctrl+C raises KeyboardInterrupt to stop."""
    period_s = 1.0 / hz
    while True:
        callback()
        time.sleep(period_s)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def choose_approach_vector():
    return {
        "name": "local_north_negative",
        "north_sign": -1.0,
        "east_sign": 0.0,
        "lateral_axis": "east",
    }


def approach_until_target(master, boot_time, args, x0, y0, fov_monitor=None, distance_monitor=None):
    monitor = distance_monitor or ActualDistanceMonitor()
    owns_monitor = distance_monitor is None
    if owns_monitor:
        monitor.start()
    period_s = 1.0 / args.setpoint_hz
    deadline = time.monotonic() + args.approach_timeout_s
    last_report = 0.0
    last_actual = None
    initial_actual = None
    best_actual = None
    approach = choose_approach_vector()
    max_lateral_mps = min(5.0, args.speed_mps * 0.5)
    lateral_gain = 0.08
    approach_speed_mps = args.speed_mps
    approach_start_monotonic = time.monotonic()

    print(
        f"Moving along {approach['name']} toward the cameras until actual distance <= {args.target_distance_m:.1f} m "
        f"(fallback max displacement {args.distance_m:.1f} m, lateral bias {args.lateral_bias_m:.1f} m, "
        f"lateral sweep +/-{args.lateral_sweep_amplitude_m:.1f} m every {args.lateral_sweep_period_s:.1f} s) ...",
        flush=True,
    )
    try:
        while time.monotonic() < deadline:
            msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            traveled = None
            lateral_error = 0.0
            lateral_reference = args.lateral_bias_m + triangle_wave(
                time.monotonic() - approach_start_monotonic,
                args.lateral_sweep_period_s,
                args.lateral_sweep_amplitude_m,
            )
            if msg is not None:
                x = float(msg.x)
                y = float(msg.y)
                traveled = math.hypot(x - x0, y - y0)
                raw_lateral = (x - x0) if approach["lateral_axis"] == "north" else (y - y0)
                lateral_error = raw_lateral - lateral_reference

            fov_allowed = True
            fov_reason = "FOV guard disabled"
            if fov_monitor is not None:
                fov_allowed, fov_reason = fov_monitor.snapshot()
            if fov_allowed:
                lateral_mps = clamp(-lateral_gain * lateral_error, -max_lateral_mps, max_lateral_mps)
                north_mps = approach["north_sign"] * approach_speed_mps
                east_mps = approach["east_sign"] * approach_speed_mps
                if approach["lateral_axis"] == "north":
                    north_mps += lateral_mps
                else:
                    east_mps += lateral_mps
            else:
                # Once both bboxes are known, stop on a margin violation. A
                # missing/stale detection is intentionally handled as an
                # ungated condition by BboxFovMonitor.snapshot().
                lateral_mps = 0.0
                north_mps = 0.0
                east_mps = 0.0
            send_local_velocity_setpoint(
                master,
                boot_time,
                north_mps=north_mps,
                east_mps=east_mps,
            )

            actual = monitor.value
            if actual is not None:
                last_actual = actual
                if initial_actual is None:
                    initial_actual = actual
                    best_actual = actual
                if actual <= args.target_distance_m:
                    print(f"Target actual distance reached: {actual:.1f} m", flush=True)
                    return
                closing_distance = actual - args.target_distance_m
                if closing_distance < 200.0:
                    approach_speed_mps = clamp(closing_distance * 0.12, 2.0, args.speed_mps)
                else:
                    approach_speed_mps = args.speed_mps
                if best_actual is None or actual < best_actual:
                    best_actual = actual
            elif traveled is not None and traveled >= args.distance_m:
                print(
                    f"Fallback displacement reached without actual-distance topic: "
                    f"{traveled:.1f} m",
                    flush=True,
                )
                return

            if time.monotonic() - last_report > 5.0:
                actual_text = "n/a" if last_actual is None else f"{last_actual:.1f} m"
                traveled_text = "n/a" if traveled is None else f"{traveled:.1f} m"
                print(
                    f"  actual={actual_text}, traveled={traveled_text}, "
                    f"speed={approach_speed_mps:.1f} m/s, "
                    f"lateral_error={lateral_error:.1f} m, lateral_reference={lateral_reference:.1f} m, "
                    f"lateral={lateral_mps:.1f} m/s, "
                    f"fov_allowed={fov_allowed}, fov_reason={fov_reason}",
                    flush=True,
                )
                last_report = time.monotonic()
            time.sleep(period_s)
    finally:
        send_local_velocity_setpoint(master, boot_time, 0.0, 0.0, 0.0)
        if owns_monitor:
            monitor.stop()

    raise RuntimeError(
        f"Approach timed out before reaching {args.target_distance_m:.1f} m actual distance."
    )


def follow_waypoints_until_margin_filled(
    master, boot_time, args, waypoints, fov_monitor, distance_monitor=None
):
    """Fly the given waypoints (takeoff included), gated by the FOV margin.

    Moves toward each waypoint in turn using a 3D velocity vector, same as
    ``px4_waypoint_mission.py``'s ``wait_waypoint``. On a hard margin
    violation (a bbox edge has crossed past the margin) it freezes, same as
    ``approach_until_target``. Its normal exit is different: the instant both
    cameras' bboxes span the full safe box on every side (see
    ``BboxFovMonitor.filled_snapshot``), it stops right there and holds
    position in OFFBOARD forever (Ctrl+C to stop) instead of continuing to
    the next waypoint -- getting closer from there would only push the bbox
    past the margin.

    Every exit path -- margin filled, a waypoint/approach timeout, or an
    exception -- zeroes velocity before returning/raising so a stale non-zero
    setpoint is never the last thing PX4 heard from this process.
    """

    period_s = 1.0 / args.setpoint_hz
    deadline = time.monotonic() + args.approach_timeout_s
    last_report = 0.0
    last_pos = None

    try:
        for index, (x, y, z) in enumerate(waypoints, start=1):
            print(
                f"Waypoint {index}/{len(waypoints)}: north={x:.1f}, east={y:.1f}, "
                f"altitude={-z:.1f} m",
                flush=True,
            )
            while time.monotonic() < deadline:
                msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                if msg is not None:
                    last_pos = msg

                filled, filled_reason = fov_monitor.filled_snapshot()
                if filled:
                    hold_x = float(last_pos.x) if last_pos is not None else x
                    hold_y = float(last_pos.y) if last_pos is not None else y
                    hold_z = float(last_pos.z) if last_pos is not None else z
                    print(
                        f"Safety margin filled on every side ({filled_reason}); "
                        f"holding position north={hold_x:.1f}, east={hold_y:.1f}, "
                        f"altitude={-hold_z:.1f} m in OFFBOARD (Ctrl+C to stop).",
                        flush=True,
                    )
                    if distance_monitor is not None:
                        distance_monitor.publish_phase("margin_filled_hover")
                    stream_until_interrupted(
                        args.setpoint_hz,
                        lambda: send_position_setpoint(master, boot_time, hold_x, hold_y, hold_z),
                    )
                    return

                fov_allowed, fov_reason = fov_monitor.snapshot()
                remaining = None
                if last_pos is not None:
                    remaining = math.sqrt(
                        (x - float(last_pos.x)) ** 2
                        + (y - float(last_pos.y)) ** 2
                        + (z - float(last_pos.z)) ** 2
                    )
                if fov_allowed and remaining is not None and remaining > 0.001:
                    speed = min(args.speed_mps, remaining)
                    north_mps = (x - float(last_pos.x)) / remaining * speed
                    east_mps = (y - float(last_pos.y)) / remaining * speed
                    down_mps = (z - float(last_pos.z)) / remaining * speed
                else:
                    north_mps = east_mps = down_mps = 0.0
                send_local_velocity_setpoint(
                    master, boot_time, north_mps=north_mps, east_mps=east_mps, down_mps=down_mps
                )

                if time.monotonic() - last_report > 2.0:
                    remaining_text = "n/a" if remaining is None else f"{remaining:.1f} m"
                    print(
                        f"  remaining={remaining_text}, "
                        f"velocity=({north_mps:.1f}, {east_mps:.1f}, {down_mps:.1f}) m/s, "
                        f"fov_allowed={fov_allowed}, fov_reason={fov_reason}",
                        flush=True,
                    )
                    last_report = time.monotonic()

                if remaining is not None and remaining <= args.waypoint_acceptance_radius_m:
                    print(f"Waypoint {index} reached.", flush=True)
                    break
                time.sleep(period_s)
            else:
                raise RuntimeError(
                    f"Approach timed out before waypoint {index} was reached "
                    "and the safety margin was never filled."
                )

        raise RuntimeError(
            "All waypoints were flown but the safety margin was never filled."
        )
    finally:
        send_local_velocity_setpoint(master, boot_time, 0.0, 0.0, 0.0)


def wait_takeoff(master, boot_time, hz, x, y, target_z, timeout_s):
    print(f"Taking off to {-target_z:.1f} m AGL ...", flush=True)
    period_s = 1.0 / hz
    deadline = time.monotonic() + timeout_s
    last_report = 0.0
    while time.monotonic() < deadline:
        send_position_setpoint(master, boot_time, x, y, target_z)
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg is not None:
            altitude = -float(msg.z)
            if time.monotonic() - last_report > 2.0:
                print(f"  altitude={altitude:.1f} m", flush=True)
                last_report = time.monotonic()
            if math.isfinite(altitude) and altitude >= (-target_z - 0.5):
                print("Takeoff altitude reached.", flush=True)
                return
        time.sleep(period_s)
    raise RuntimeError("Takeoff timed out before reaching target altitude.")


def wait_landed(master, timeout_s):
    print("Waiting for landing ...", flush=True)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = master.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("Landed.", flush=True)
            return
    raise RuntimeError("Landing timeout expired.")


def run(args):
    master = mavutil.mavlink_connection(args.connect, autoreconnect=True)
    fov_monitor = None
    heartbeat = None
    distance_monitor = None
    try:
        wait_heartbeat(master, args.heartbeat_timeout_s)
        heartbeat = GcsHeartbeat(master)
        heartbeat.start()
        distance_monitor = ActualDistanceMonitor()
        distance_monitor.start()
        distance_monitor.publish_phase("preflight")
        if args.fov_guard:
            fov_monitor = BboxFovMonitor(
                topic=os.environ.get("AUTO_FOV_DETECTIONS_TOPIC", "/stereo/live_detections"),
                image_width_px=int(os.environ.get("CAMERA_IMAGE_WIDTH_PX", "1280")),
                image_height_px=int(os.environ.get("CAMERA_IMAGE_HEIGHT_PX", "720")),
                margin_ratio=args.bbox_margin_ratio,
                timeout_s=args.fov_detection_timeout_s,
            )
            fov_monitor.start()
            if fov_monitor.error:
                raise RuntimeError(fov_monitor.error)
        pos = wait_local_position(master)
        if args.check_connection:
            print(
                "PX4 local position: "
                f"x={float(pos.x):.2f}, y={float(pos.y):.2f}, z={float(pos.z):.2f}",
                flush=True,
            )
            print("Connection check complete. No mission command was sent.", flush=True)
            return
        if args.mpc_z_vel_max_up_mps is not None:
            set_param_float(master, "MPC_Z_VEL_MAX_UP", args.mpc_z_vel_max_up_mps)
        if args.mpc_z_vel_max_dn_mps is not None:
            set_param_float(master, "MPC_Z_VEL_MAX_DN", args.mpc_z_vel_max_dn_mps)
        x0 = float(pos.x)
        y0 = float(pos.y)
        boot_time = time.monotonic()

        waypoints = []
        if args.waypoints_xy_alt:
            waypoints += parse_waypoint_list(args.waypoints_xy_alt)
        if args.path_file:
            waypoints += read_path_file(args.path_file)
        waypoints += list(args.waypoint)
        if waypoints:
            if fov_monitor is None:
                raise RuntimeError("--waypoints-xy-alt/--waypoint requires --fov-guard")
            first_x, first_y, first_z = waypoints[0]
            # PX4 requires a short stream of setpoints before OFFBOARD mode is accepted.
            stream_for(
                2.0,
                args.setpoint_hz,
                lambda: send_position_setpoint(master, boot_time, first_x, first_y, first_z),
            )
            distance_monitor.publish_phase("takeoff")
            set_mode(master, "OFFBOARD")
            stream_for(
                2.0,
                args.setpoint_hz,
                lambda: send_position_setpoint(master, boot_time, first_x, first_y, first_z),
            )
            arm(master)
            distance_monitor.publish_phase("approach")
            follow_waypoints_until_margin_filled(
                master, boot_time, args, waypoints, fov_monitor,
                distance_monitor=distance_monitor,
            )
            return

        target_z = -float(args.altitude_m)

        # PX4 requires a short stream of setpoints before OFFBOARD mode is accepted.
        stream_for(
            2.0,
            args.setpoint_hz,
            lambda: send_position_setpoint(master, boot_time, x0, y0, target_z),
        )
        distance_monitor.publish_phase("takeoff")
        set_mode(master, "OFFBOARD")
        stream_for(
            2.0,
            args.setpoint_hz,
            lambda: send_position_setpoint(master, boot_time, x0, y0, target_z),
        )
        arm(master)

        wait_takeoff(
            master,
            boot_time,
            args.setpoint_hz,
            x0,
            y0,
            target_z,
            args.takeoff_timeout_s,
        )
        distance_monitor.publish_phase("hover_before_approach")
        stream_for(
            args.first_hover_s,
            args.setpoint_hz,
            lambda: send_position_setpoint(master, boot_time, x0, y0, target_z),
        )

        distance_monitor.publish_phase("approach")
        approach_until_target(
            master, boot_time, args, x0, y0,
            fov_monitor=fov_monitor, distance_monitor=distance_monitor,
        )

        print("Final hover ...", flush=True)
        distance_monitor.publish_phase("hover_after_approach")
        stream_for(
            args.final_hover_s,
            args.setpoint_hz,
            lambda: send_body_velocity_setpoint(master, boot_time, 0.0),
        )

        print("Landing ...", flush=True)
        distance_monitor.publish_phase("landing")
        request_land(master)
        wait_landed(master, args.land_timeout_s)
        distance_monitor.publish_phase("complete")
    except (KeyboardInterrupt, Exception):
        # A GUI Stop, terminal Ctrl+C, or any mission failure (e.g. an
        # approach/waypoint timeout) must not leave an armed vehicle in
        # OFFBOARD with no more setpoints coming -- that just hands control
        # to PX4's own offboard-loss failsafe (RTL by default with no RC),
        # which looks like the drone reversing course on its own.
        try:
            if distance_monitor is not None:
                distance_monitor.publish_phase("interrupted")
            request_land(master)
            print("Mission stopping; LAND requested.", flush=True)
        except Exception as exc:
            print(f"Mission stopping; LAND request failed: {exc}", file=sys.stderr)
        raise
    finally:
        if distance_monitor is not None:
            distance_monitor.stop()
        if fov_monitor is not None:
            fov_monitor.stop()
        if heartbeat is not None:
            heartbeat.stop()


def main():
    args = build_parser().parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("Mission interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Mission failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
