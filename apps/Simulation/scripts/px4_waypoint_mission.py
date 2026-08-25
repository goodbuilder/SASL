#!/usr/bin/env python3
import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

from px4_auto_mission import (
    ActualDistanceMonitor,
    GcsHeartbeat,
    arm,
    command_long,
    now_ms,
    positive_float,
    send_local_velocity_setpoint,
    send_position_setpoint,
    set_mode,
    set_param_float,
    stream_for,
    stream_until_interrupted,
    wait_heartbeat,
    wait_landed,
    wait_local_position,
)


STABILITY_SCHEMA_VERSION = "px4_waypoint_stability_v2"
GUIDANCE_SCHEMA_VERSION = "px4_waypoint_guidance_v1"
DEFAULT_GUIDANCE_MODE = "position_velocity_final_target"
GUIDANCE_MODES = (
    "position_velocity_final_target",
    "position_only_final_target",
    "velocity_only",
)


def non_negative_float(value):
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def parse_xy_alt(entry):
    parts = [part.strip() for part in entry.split(",")]
    if len(parts) not in (3, 4):
        raise argparse.ArgumentTypeError(
            "waypoint must be north,east,altitude_agl_m[,hold_s]"
        )
    north, east, altitude = (float(part) for part in parts[:3])
    if altitude < 0:
        raise argparse.ArgumentTypeError("altitude_agl_m must be zero or greater")
    hold_s = None
    if len(parts) == 4 and parts[3] != "":
        hold_s = float(parts[3])
        if hold_s < 0:
            raise argparse.ArgumentTypeError("hold_s must be zero or greater")
    return north, east, -altitude, hold_s


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
                hold_s = row.get("hold_s") or row.get("hold")
                entry = f"{north},{east},{altitude}"
                if hold_s not in (None, ""):
                    entry += f",{hold_s}"
                waypoints.append(parse_xy_alt(entry))
        else:
            reader = csv.reader(
                row for row in handle if row.strip() and not row.lstrip().startswith("#")
            )
            for row in reader:
                waypoints.append(parse_xy_alt(",".join(row)))
    return waypoints


def build_parser():
    parser = argparse.ArgumentParser(
        description="Run a PX4 offboard waypoint mission using LOCAL_NED x/y and positive AGL altitude inputs."
    )
    parser.add_argument(
        "--connect",
        default="udpin:0.0.0.0:14540",
        help="pymavlink connection string. Default: udpin:0.0.0.0:14540",
    )
    parser.add_argument(
        "--waypoints-xy-alt",
        default="",
        help="Semicolon-separated north,east,altitude_agl_m[,hold_s] list. "
        "hold_s overrides --hold-s for that one waypoint (e.g. to hover "
        "only at some waypoints in a dense path); omit it to use --hold-s.",
    )
    parser.add_argument(
        "--waypoint",
        action="append",
        default=[],
        type=parse_xy_alt,
        metavar="X,Y,ALT[,HOLD_S]",
        help="Add one north,east,altitude_agl_m[,hold_s] waypoint. Repeatable.",
    )
    parser.add_argument("--path-file", help="Read waypoints from CSV/text file.")
    parser.add_argument("--speed-mps", type=positive_float, default=12.0)
    parser.add_argument(
        "--guidance-mode",
        choices=GUIDANCE_MODES,
        default=DEFAULT_GUIDANCE_MODE,
        help=(
            "Transit setpoint contract. position_velocity_final_target sends "
            "the final position plus velocity feed-forward; "
            "position_only_final_target removes only that feed-forward for a "
            "controlled diagnostic comparison; velocity_only ignores the far "
            "final position during transit and sends only the pure-pursuit "
            "local-NED velocity."
        ),
    )
    parser.add_argument(
        "--mpc-z-vel-max-up-mps",
        type=positive_float,
        default=None,
        help="Override PX4 MPC_Z_VEL_MAX_UP (climb speed limit, default PX4 value: 3 m/s) "
        "before the mission starts. Unset leaves PX4's configured value alone.",
    )
    parser.add_argument(
        "--mpc-z-vel-max-dn-mps",
        type=positive_float,
        default=None,
        help="Override PX4 MPC_Z_VEL_MAX_DN (descent speed limit, default PX4 value: 1.5 m/s) "
        "before the mission starts. Unset leaves PX4's configured value alone.",
    )
    parser.add_argument(
        "--mpc-tiltmax-air-deg",
        type=positive_float,
        default=None,
        help="Override PX4 MPC_TILTMAX_AIR (max roll/pitch lean angle in flight, "
        "default PX4 value: 45 deg) before the mission starts. Unset leaves "
        "PX4's configured value alone.",
    )
    parser.add_argument(
        "--mc-rollrate-p",
        type=positive_float,
        default=None,
        help="Override PX4 MC_ROLLRATE_P (roll rate loop gain, default PX4 value: "
        "0.15) before the mission starts. Higher values reach the commanded "
        "tilt rate faster. Unset leaves PX4's configured value alone.",
    )
    parser.add_argument(
        "--mc-pitchrate-p",
        type=positive_float,
        default=None,
        help="Override PX4 MC_PITCHRATE_P (pitch rate loop gain, default PX4 "
        "value: 0.15) before the mission starts. Unset leaves PX4's "
        "configured value alone.",
    )
    parser.add_argument(
        "--mc-roll-p",
        type=positive_float,
        default=None,
        help="Override PX4 MC_ROLL_P (roll attitude loop gain, default PX4 "
        "value: 4.0) before the mission starts. Unset leaves PX4's "
        "configured value alone.",
    )
    parser.add_argument(
        "--mc-pitch-p",
        type=positive_float,
        default=None,
        help="Override PX4 MC_PITCH_P (pitch attitude loop gain, default PX4 "
        "value: 4.0) before the mission starts. Unset leaves PX4's "
        "configured value alone.",
    )
    parser.add_argument(
        "--mpc-xy-vel-p-acc",
        type=positive_float,
        default=None,
        help="Override PX4 MPC_XY_VEL_P_ACC (horizontal velocity loop gain, "
        "default PX4 value: 1.8) before the mission starts. Unset leaves "
        "PX4's configured value alone.",
    )
    parser.add_argument(
        "--mpc-acc-hor-max",
        type=positive_float,
        default=None,
        help="Override PX4 MPC_ACC_HOR_MAX (hard horizontal acceleration cap, "
        "default PX4 value: 5.0 m/s^2) before the mission starts. Unlike the "
        "loop gains, this is a ceiling the controller saturates against, not "
        "an error multiplier. Unset leaves PX4's configured value alone.",
    )
    parser.add_argument("--acceptance-radius-m", type=positive_float, default=5.0)
    parser.add_argument("--waypoint-timeout-s", type=positive_float, default=120.0)
    parser.add_argument(
        "--hold-s",
        type=non_negative_float,
        default=1.0,
        help="Hover time at each waypoint, in seconds. 0 disables the hover.",
    )
    parser.add_argument(
        "--ramp-s",
        type=non_negative_float,
        default=1.0,
        help="Time to accelerate the horizontal speed from 0 to full "
        "speed_mps (and symmetrically decelerate to 0 on arrival) at the "
        "start of a leg, in seconds. Direction is fixed at the start of "
        "the leg rather than re-aimed from the live position every tick, "
        "so this and --vertical-ramp-s independently time the horizontal "
        "and vertical components of one straight-line path instead of "
        "each other's noise. 0 disables ramping (jumps straight to full "
        "speed).",
    )
    parser.add_argument(
        "--vertical-ramp-s",
        type=non_negative_float,
        default=2.5,
        help="Time to accelerate the vertical speed from 0 to full "
        "speed_mps, in seconds. Kept longer than --ramp-s by default: the "
        "vehicle's real Z response (thrust only) engages almost "
        "immediately while its horizontal response lags (needs to build a "
        "tilt angle first), so ramping vertical in more slowly keeps it "
        "from outrunning horizontal and dropping off the intended "
        "approach line. 0 disables ramping.",
    )
    parser.add_argument(
        "--final-hover-s",
        type=non_negative_float,
        default=3.0,
        help="Hover time after the final waypoint, in seconds. 0 disables the hover.",
    )
    parser.add_argument("--land-timeout-s", type=positive_float, default=90.0)
    parser.add_argument("--setpoint-hz", type=positive_float, default=20.0)
    parser.add_argument("--event-log", type=Path, default=None)
    parser.add_argument(
        "--guidance-log",
        type=Path,
        default=None,
        help=(
            "Write every transit/hold setpoint and its LOCAL_POSITION_NED "
            "readback as JSONL evidence."
        ),
    )
    parser.add_argument(
        "--stability-window-s",
        type=positive_float,
        default=3.0,
        help="Tail window of each waypoint hold used for stability metrics.",
    )
    parser.add_argument(
        "--stability-position-p95-max-m",
        type=positive_float,
        default=None,
        help="Fail a hold when its tail-window 3D position-error p95 exceeds this value.",
    )
    parser.add_argument(
        "--stability-attitude-max-deg",
        type=positive_float,
        default=None,
        help="Fail a hold when absolute roll or pitch exceeds this value in the tail window.",
    )
    parser.add_argument("--heartbeat-timeout-s", type=positive_float, default=30.0)
    parser.add_argument(
        "--land-after",
        dest="land_after",
        action="store_true",
        help="Land after the final waypoint.",
    )
    parser.add_argument(
        "--no-land",
        dest="land_after",
        action="store_false",
        help="Stay in offboard hover after the final waypoint.",
    )
    parser.set_defaults(land_after=True)
    parser.add_argument(
        "--stop-after-hover",
        dest="stop_after_hover",
        action="store_true",
        help="With --no-land: exit automatically once --final-hover-s "
        "elapses, instead of hovering indefinitely until interrupted "
        "(Ctrl+C). Ignored when landing after the final waypoint, since "
        "landing already ends the mission on its own.",
    )
    parser.add_argument(
        "--no-stop-after-hover",
        dest="stop_after_hover",
        action="store_false",
        help="With --no-land: hover indefinitely at the final waypoint "
        "until interrupted (Ctrl+C). This is the default.",
    )
    parser.set_defaults(stop_after_hover=False)
    parser.add_argument(
        "--check-connection",
        action="store_true",
        help="Connect to PX4 and print local position without arming or moving.",
    )
    return parser


def collect_waypoints(args):
    waypoints = []
    if args.waypoints_xy_alt:
        waypoints.extend(parse_waypoint_list(args.waypoints_xy_alt))
    if args.path_file:
        waypoints.extend(read_path_file(args.path_file))
    waypoints.extend(args.waypoint)
    if not waypoints and not args.check_connection:
        raise RuntimeError(
            "No waypoints configured. Edit config/simulation.env or pass --waypoint."
        )
    return waypoints


def distance_3d(pos, waypoint):
    return math.sqrt(
        (float(pos.x) - waypoint[0]) ** 2
        + (float(pos.y) - waypoint[1]) ** 2
        + (float(pos.z) - waypoint[2]) ** 2
    )


def _percentile_nearest_rank(values, percentile):
    if not values:
        return math.nan
    ordered = sorted(float(value) for value in values)
    index = max(0, min(len(ordered) - 1, math.ceil(percentile * len(ordered)) - 1))
    return ordered[index]


def _append_event(path, payload, *, create=False):
    if path is None:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = "x" if create else "a"
    with path.open(mode, encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")


def _event_clock():
    return {
        "event_time_unix_s": time.time(),
        "event_time_monotonic_s": time.monotonic(),
    }


def _waypoint_phase(index, state):
    return f"waypoint_{int(index):03d}_{state}"


def _publish_phase(phase_monitor, phase):
    if phase_monitor is not None:
        phase_monitor.publish_phase(phase)


def _message_vector(message, names):
    if message is None:
        return None
    values = []
    for name in names:
        try:
            value = float(getattr(message, name))
        except (AttributeError, TypeError, ValueError):
            return None
        values.append(value if math.isfinite(value) else None)
    return values


def _position_only_type_mask():
    return (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )


def _position_velocity_type_mask():
    return (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )


def _velocity_only_type_mask():
    return (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )


class GuidanceTrace:
    def __init__(self, path):
        self.path = Path(path) if path is not None else None
        self.handle = None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.handle = self.path.open("x", encoding="utf-8")

    def append(self, payload):
        if self.handle is None:
            return
        self.handle.write(
            json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n"
        )

    def close(self):
        if self.handle is not None:
            self.handle.flush()
            self.handle.close()
            self.handle = None


def _trace_setpoint(
    trace,
    *,
    guidance_mode,
    phase,
    index,
    total,
    active_fields,
    target_position,
    target_velocity,
    actual_position,
    remaining_distance_m=None,
    horizontal_ramp=None,
    vertical_ramp=None,
):
    if trace is None:
        return
    payload = {
        "schema_version": GUIDANCE_SCHEMA_VERSION,
        "event": "setpoint_sent",
        "guidance_mode": guidance_mode,
        "mission_phase": phase,
        "waypoint_index": int(index),
        "waypoint_total": int(total),
        "frame": "MAV_FRAME_LOCAL_NED",
        "active_fields": list(active_fields),
        "type_mask": (
            _position_velocity_type_mask()
            if "position" in active_fields and "velocity" in active_fields
            else _velocity_only_type_mask()
            if "velocity" in active_fields
            else _position_only_type_mask()
        ),
        "navigation_target_local_ned_m": [float(value) for value in target_position],
        # Compatibility alias: this is the navigation goal, not necessarily
        # an active MAVLink position field (see active_fields and the raw
        # mavlink_position_fields_local_ned_m evidence below).
        "target_position_local_ned_m": [float(value) for value in target_position],
        "mavlink_position_fields_local_ned_m": (
            [float(value) for value in target_position]
            if "position" in active_fields
            else [0.0, 0.0, 0.0]
        ),
        "target_velocity_local_ned_m_s": (
            None
            if target_velocity is None
            else [float(value) for value in target_velocity]
        ),
        "actual_position_local_ned_m": _message_vector(
            actual_position, ("x", "y", "z")
        ),
        "actual_velocity_local_ned_m_s": _message_vector(
            actual_position, ("vx", "vy", "vz")
        ),
        "remaining_distance_m": (
            None if remaining_distance_m is None else float(remaining_distance_m)
        ),
        "horizontal_ramp": None if horizontal_ramp is None else float(horizontal_ramp),
        "vertical_ramp": None if vertical_ramp is None else float(vertical_ramp),
        **_event_clock(),
    }
    trace.append(payload)


def hold_waypoint(
    master,
    boot_time,
    args,
    waypoint,
    hold_s,
    index,
    total,
    *,
    phase_monitor=None,
    guidance_trace=None,
):
    period_s = 1.0 / args.setpoint_hz
    start = time.monotonic()
    start_wall = time.time()
    tail_start = max(0.0, float(hold_s) - float(args.stability_window_s))
    position_errors = []
    abs_roll_deg = []
    abs_pitch_deg = []
    analysis_started_wall = None
    analysis_started_monotonic = None
    _append_event(
        args.event_log,
        {
            "schema_version": STABILITY_SCHEMA_VERSION,
            "event": "waypoint_hold_started",
            "waypoint_index": index,
            "waypoint_total": total,
            "target_local_ned_m": [float(value) for value in waypoint[:3]],
            "hold_s": float(hold_s),
            "stability_window_s": min(float(hold_s), float(args.stability_window_s)),
            **_event_clock(),
        },
    )
    while time.monotonic() - start < hold_s:
        send_position_setpoint(master, boot_time, waypoint[0], waypoint[1], waypoint[2])
        elapsed = time.monotonic() - start
        analysis_active = elapsed >= tail_start
        phase = _waypoint_phase(
            index, "hold_analysis" if analysis_active else "hold_settle"
        )
        _publish_phase(phase_monitor, phase)
        pos = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        attitude = master.recv_match(type="ATTITUDE", blocking=False)
        _trace_setpoint(
            guidance_trace,
            guidance_mode=args.guidance_mode,
            phase=phase,
            index=index,
            total=total,
            active_fields=("position",),
            target_position=waypoint[:3],
            target_velocity=None,
            actual_position=pos,
            remaining_distance_m=(distance_3d(pos, waypoint) if pos is not None else None),
        )
        if analysis_active:
            if analysis_started_wall is None:
                analysis_started_wall = time.time()
                analysis_started_monotonic = time.monotonic()
                _append_event(
                    args.event_log,
                    {
                        "schema_version": STABILITY_SCHEMA_VERSION,
                        "event": "waypoint_hold_analysis_started",
                        "waypoint_index": index,
                        "waypoint_total": total,
                        "target_local_ned_m": [float(value) for value in waypoint[:3]],
                        **_event_clock(),
                    },
                )
            if pos is not None:
                position_errors.append(distance_3d(pos, waypoint))
            if attitude is not None:
                abs_roll_deg.append(abs(math.degrees(float(attitude.roll))))
                abs_pitch_deg.append(abs(math.degrees(float(attitude.pitch))))
        time.sleep(period_s)

    metrics = {
        "schema_version": STABILITY_SCHEMA_VERSION,
        "event": "waypoint_hold_complete",
        "waypoint_index": index,
        "waypoint_total": total,
        "target_local_ned_m": [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])],
        "hold_s": float(hold_s),
        "stability_window_s": min(float(hold_s), float(args.stability_window_s)),
        "position_sample_count": len(position_errors),
        "attitude_sample_count": min(len(abs_roll_deg), len(abs_pitch_deg)),
        "position_error_p95_m": _percentile_nearest_rank(position_errors, 0.95),
        "max_abs_roll_deg": max(abs_roll_deg, default=math.nan),
        "max_abs_pitch_deg": max(abs_pitch_deg, default=math.nan),
        "hold_started_unix_s": start_wall,
        "hold_completed_unix_s": time.time(),
        "analysis_started_unix_s": analysis_started_wall,
        "analysis_started_monotonic_s": analysis_started_monotonic,
        **_event_clock(),
    }
    failures = []
    if args.stability_position_p95_max_m is not None:
        if not position_errors:
            failures.append("missing_position_samples")
        elif metrics["position_error_p95_m"] > args.stability_position_p95_max_m:
            failures.append("position_error_p95_exceeded")
    if args.stability_attitude_max_deg is not None:
        if not abs_roll_deg or not abs_pitch_deg:
            failures.append("missing_attitude_samples")
        elif (
            metrics["max_abs_roll_deg"] > args.stability_attitude_max_deg
            or metrics["max_abs_pitch_deg"] > args.stability_attitude_max_deg
        ):
            failures.append("attitude_limit_exceeded")
    metrics["stability_pass"] = not failures
    metrics["stability_failures"] = failures
    _append_event(args.event_log, metrics)
    if failures:
        raise RuntimeError(
            f"Waypoint {index} stability gate failed: {','.join(failures)}"
        )
    return metrics


def send_position_velocity_setpoint(master, boot_time, x, y, z, vx, vy, vz):
    type_mask = _position_velocity_type_mask()
    master.mav.set_position_target_local_ned_send(
        now_ms(boot_time),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x,
        y,
        z,
        vx,
        vy,
        vz,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def wait_waypoint(
    master,
    boot_time,
    args,
    waypoint,
    index,
    total,
    *,
    phase_monitor=None,
    guidance_trace=None,
):
    x, y, z = waypoint[:3]
    period_s = 1.0 / args.setpoint_hz
    start = time.monotonic()
    deadline = start + args.waypoint_timeout_s
    # A hold ends at zero velocity. When the next leg's velocity command
    # starts, the real vehicle's Z response (thrust only) engages almost
    # immediately, while its horizontal response (needs to build a tilt
    # angle first) lags by roughly 1.5-2s before it starts translating.
    # Commanding both axes with the same ramp does not fix this -- the
    # imbalance is in the vehicle's physical response, not the command
    # timing -- so vz is ramped in on its own, slower schedule to stay
    # synchronized with how fast the vehicle actually starts moving
    # horizontally, instead of outrunning it and dropping off the ray.
    #
    # A fixed-direction, time-explicit trapezoidal profile (single shared
    # ramp, and later two independently-ramped horizontal/vertical
    # profiles) was tried here empirically and measured worse in both
    # forms -- the live-position re-aiming below, imperfect as it is,
    # outperformed both redesigns. Left as pure pursuit.
    horizontal_ramp_s = args.ramp_s
    vertical_ramp_s = args.vertical_ramp_s
    last_report = 0.0
    print(
        f"Waypoint {index}/{total}: north={x:.1f}, east={y:.1f}, altitude={-z:.1f} m",
        flush=True,
    )
    transit_phase = _waypoint_phase(index, "transit")
    _append_event(
        args.event_log,
        {
            "schema_version": STABILITY_SCHEMA_VERSION,
            "event": "waypoint_transit_started",
            "guidance_mode": args.guidance_mode,
            "waypoint_index": index,
            "waypoint_total": total,
            "target_local_ned_m": [float(value) for value in waypoint[:3]],
            "requested_speed_mps": float(args.speed_mps),
            "horizontal_ramp_s": float(args.ramp_s),
            "vertical_ramp_s": float(args.vertical_ramp_s),
            **_event_clock(),
        },
    )
    while time.monotonic() < deadline:
        _publish_phase(phase_monitor, transit_phase)
        pos = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if pos is not None:
            remaining = distance_3d(pos, waypoint)
            h_ramp = 1.0
            v_ramp = 1.0
            if remaining > 0.001:
                speed = min(args.speed_mps, remaining)
                elapsed = time.monotonic() - start
                h_ramp = (
                    1.0 if horizontal_ramp_s <= 0 else min(1.0, elapsed / horizontal_ramp_s)
                )
                v_ramp = (
                    1.0 if vertical_ramp_s <= 0 else min(1.0, elapsed / vertical_ramp_s)
                )
                vx = (x - float(pos.x)) / remaining * speed * h_ramp
                vy = (y - float(pos.y)) / remaining * speed * h_ramp
                vz = (z - float(pos.z)) / remaining * speed * v_ramp
            else:
                vx = vy = vz = 0.0
            if args.guidance_mode == "position_only_final_target":
                send_position_setpoint(master, boot_time, x, y, z)
                active_fields = ("position",)
                traced_velocity = None
            elif args.guidance_mode == "velocity_only":
                send_local_velocity_setpoint(master, boot_time, vx, vy, vz)
                active_fields = ("velocity",)
                traced_velocity = (vx, vy, vz)
            else:
                send_position_velocity_setpoint(master, boot_time, x, y, z, vx, vy, vz)
                active_fields = ("position", "velocity")
                traced_velocity = (vx, vy, vz)
            _trace_setpoint(
                guidance_trace,
                guidance_mode=args.guidance_mode,
                phase=transit_phase,
                index=index,
                total=total,
                active_fields=active_fields,
                target_position=(x, y, z),
                target_velocity=traced_velocity,
                actual_position=pos,
                remaining_distance_m=remaining,
                horizontal_ramp=h_ramp,
                vertical_ramp=v_ramp,
            )
            if time.monotonic() - last_report > 2.0:
                print(f"  remaining={remaining:.1f} m", flush=True)
                last_report = time.monotonic()
            if remaining <= args.acceptance_radius_m:
                print(f"Waypoint {index} reached.", flush=True)
                _append_event(
                    args.event_log,
                    {
                        "schema_version": STABILITY_SCHEMA_VERSION,
                        "event": "waypoint_reached",
                        "waypoint_index": index,
                        "waypoint_total": total,
                        "target_local_ned_m": [float(value) for value in waypoint[:3]],
                        "actual_local_ned_m": _message_vector(pos, ("x", "y", "z")),
                        "actual_velocity_local_ned_m_s": _message_vector(
                            pos, ("vx", "vy", "vz")
                        ),
                        "remaining_distance_m": float(remaining),
                        **_event_clock(),
                    },
                )
                return
        else:
            send_position_setpoint(master, boot_time, x, y, z)
            _trace_setpoint(
                guidance_trace,
                guidance_mode=args.guidance_mode,
                phase=transit_phase,
                index=index,
                total=total,
                active_fields=("position",),
                target_position=(x, y, z),
                target_velocity=None,
                actual_position=None,
            )
        time.sleep(period_s)
    raise RuntimeError(f"Waypoint {index} timed out.")


def run(args):
    waypoints = collect_waypoints(args)
    guidance_trace = GuidanceTrace(args.guidance_log)
    if args.event_log is not None:
        _append_event(
            args.event_log,
            {
                "schema_version": STABILITY_SCHEMA_VERSION,
                "event": "mission_started",
                "waypoint_count": len(waypoints),
                "guidance_mode": args.guidance_mode,
                "requested_speed_mps": float(args.speed_mps),
                "horizontal_ramp_s": float(args.ramp_s),
                "vertical_ramp_s": float(args.vertical_ramp_s),
                **_event_clock(),
            },
            create=True,
        )
    master = mavutil.mavlink_connection(args.connect, autoreconnect=True)
    wait_heartbeat(master, args.heartbeat_timeout_s)
    heartbeat = GcsHeartbeat(master)
    heartbeat.start()
    phase_monitor = ActualDistanceMonitor()
    phase_monitor.start()
    _publish_phase(phase_monitor, "preflight")
    pos = wait_local_position(master)
    completed = False
    try:
        if args.check_connection:
            print(
                "PX4 local position: "
                f"x={float(pos.x):.2f}, y={float(pos.y):.2f}, z={float(pos.z):.2f}",
                flush=True,
            )
            print("Connection check complete. No mission command was sent.", flush=True)
            completed = True
            return

        if args.mpc_z_vel_max_up_mps is not None:
            set_param_float(master, "MPC_Z_VEL_MAX_UP", args.mpc_z_vel_max_up_mps)
        if args.mpc_z_vel_max_dn_mps is not None:
            set_param_float(master, "MPC_Z_VEL_MAX_DN", args.mpc_z_vel_max_dn_mps)
        if args.mpc_tiltmax_air_deg is not None:
            set_param_float(master, "MPC_TILTMAX_AIR", args.mpc_tiltmax_air_deg)
        if args.mc_rollrate_p is not None:
            set_param_float(master, "MC_ROLLRATE_P", args.mc_rollrate_p)
        if args.mc_pitchrate_p is not None:
            set_param_float(master, "MC_PITCHRATE_P", args.mc_pitchrate_p)
        if args.mc_roll_p is not None:
            set_param_float(master, "MC_ROLL_P", args.mc_roll_p)
        if args.mc_pitch_p is not None:
            set_param_float(master, "MC_PITCH_P", args.mc_pitch_p)
        if args.mpc_xy_vel_p_acc is not None:
            set_param_float(master, "MPC_XY_VEL_P_ACC", args.mpc_xy_vel_p_acc)
        if args.mpc_acc_hor_max is not None:
            set_param_float(master, "MPC_ACC_HOR_MAX", args.mpc_acc_hor_max)

        first = waypoints[0]
        boot_time = time.monotonic()
        print(f"Loaded {len(waypoints)} waypoint(s).", flush=True)

        stream_for(
            2.0,
            args.setpoint_hz,
            lambda: send_position_setpoint(master, boot_time, first[0], first[1], first[2]),
        )
        set_mode(master, "OFFBOARD")
        stream_for(
            2.0,
            args.setpoint_hz,
            lambda: send_position_setpoint(master, boot_time, first[0], first[1], first[2]),
        )
        arm(master)

        for index, waypoint in enumerate(waypoints, start=1):
            wait_waypoint(
                master,
                boot_time,
                args,
                waypoint,
                index,
                len(waypoints),
                phase_monitor=phase_monitor,
                guidance_trace=guidance_trace,
            )
            hold_s = waypoint[3] if waypoint[3] is not None else args.hold_s
            hold_waypoint(
                master,
                boot_time,
                args,
                waypoint,
                hold_s,
                index,
                len(waypoints),
                phase_monitor=phase_monitor,
                guidance_trace=guidance_trace,
            )

        print("Final hover ...", flush=True)
        _publish_phase(phase_monitor, "final_hover")
        last = waypoints[-1]
        stream_for(
            args.final_hover_s,
            args.setpoint_hz,
            lambda: send_position_setpoint(master, boot_time, last[0], last[1], last[2]),
        )

        if args.land_after:
            print("Landing ...", flush=True)
            send_local_velocity_setpoint(master, boot_time, 0.0, 0.0, 0.0)
            if "LAND" in master.mode_mapping():
                set_mode(master, "LAND")
            else:
                command_long(master, mavutil.mavlink.MAV_CMD_NAV_LAND)
            wait_landed(master, args.land_timeout_s)
        elif args.stop_after_hover:
            print("Mission complete. Stopping after the final hover.", flush=True)
        else:
            print(
                "Mission complete. Hovering at the final waypoint in OFFBOARD "
                "(Ctrl+C to stop).",
                flush=True,
            )
            stream_until_interrupted(
                args.setpoint_hz,
                lambda: send_position_setpoint(master, boot_time, last[0], last[1], last[2]),
            )
        _append_event(
            args.event_log,
            {
                "schema_version": STABILITY_SCHEMA_VERSION,
                "event": "mission_completed",
                "waypoint_count": len(waypoints),
                **_event_clock(),
            },
        )
        _publish_phase(phase_monitor, "complete")
        completed = True
    finally:
        if not completed:
            _publish_phase(phase_monitor, "interrupted")
        phase_monitor.stop()
        heartbeat.stop()
        guidance_trace.close()


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
