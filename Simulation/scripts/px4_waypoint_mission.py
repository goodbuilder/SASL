#!/usr/bin/env python3
import argparse
import csv
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

from px4_auto_mission import (
    GcsHeartbeat,
    arm,
    command_long,
    now_ms,
    positive_float,
    send_local_velocity_setpoint,
    send_position_setpoint,
    set_mode,
    stream_for,
    wait_heartbeat,
    wait_landed,
    wait_local_position,
)


def non_negative_float(value):
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def parse_xy_alt(entry):
    parts = [part.strip() for part in entry.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "waypoint must be north,east,altitude_agl_m"
        )
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
        help="Semicolon-separated north,east,altitude_agl_m list.",
    )
    parser.add_argument(
        "--waypoint",
        action="append",
        default=[],
        type=parse_xy_alt,
        metavar="X,Y,ALT",
        help="Add one north,east,altitude_agl_m waypoint. Repeatable.",
    )
    parser.add_argument("--path-file", help="Read waypoints from CSV/text file.")
    parser.add_argument("--speed-mps", type=positive_float, default=12.0)
    parser.add_argument("--acceptance-radius-m", type=positive_float, default=5.0)
    parser.add_argument("--waypoint-timeout-s", type=positive_float, default=120.0)
    parser.add_argument("--takeoff-timeout-s", type=positive_float, default=60.0)
    parser.add_argument("--hold-s", type=non_negative_float, default=1.0)
    parser.add_argument("--final-hover-s", type=non_negative_float, default=3.0)
    parser.add_argument("--land-timeout-s", type=positive_float, default=90.0)
    parser.add_argument("--setpoint-hz", type=positive_float, default=20.0)
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


def send_position_velocity_setpoint(master, boot_time, x, y, z, vx, vy, vz):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
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
        vx,
        vy,
        vz,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def wait_waypoint(master, boot_time, args, waypoint, index, total):
    x, y, z = waypoint
    period_s = 1.0 / args.setpoint_hz
    deadline = time.monotonic() + args.waypoint_timeout_s
    last_report = 0.0
    print(
        f"Waypoint {index}/{total}: north={x:.1f}, east={y:.1f}, altitude={-z:.1f} m",
        flush=True,
    )
    while time.monotonic() < deadline:
        pos = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if pos is not None:
            remaining = distance_3d(pos, waypoint)
            if remaining > 0.001:
                speed = min(args.speed_mps, remaining)
                vx = (x - float(pos.x)) / remaining * speed
                vy = (y - float(pos.y)) / remaining * speed
                vz = (z - float(pos.z)) / remaining * speed
            else:
                vx = vy = vz = 0.0
            send_position_velocity_setpoint(master, boot_time, x, y, z, vx, vy, vz)
            if time.monotonic() - last_report > 2.0:
                print(f"  remaining={remaining:.1f} m", flush=True)
                last_report = time.monotonic()
            if remaining <= args.acceptance_radius_m:
                print(f"Waypoint {index} reached.", flush=True)
                return
        else:
            send_position_setpoint(master, boot_time, x, y, z)
        time.sleep(period_s)
    raise RuntimeError(f"Waypoint {index} timed out.")


def run(args):
    waypoints = collect_waypoints(args)
    master = mavutil.mavlink_connection(args.connect, autoreconnect=True)
    wait_heartbeat(master, args.heartbeat_timeout_s)
    heartbeat = GcsHeartbeat(master)
    heartbeat.start()
    pos = wait_local_position(master)
    try:
        if args.check_connection:
            print(
                "PX4 local position: "
                f"x={float(pos.x):.2f}, y={float(pos.y):.2f}, z={float(pos.z):.2f}",
                flush=True,
            )
            print("Connection check complete. No mission command was sent.", flush=True)
            return

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
            wait_waypoint(master, boot_time, args, waypoint, index, len(waypoints))
            stream_for(
                args.hold_s,
                args.setpoint_hz,
                lambda waypoint=waypoint: send_position_setpoint(
                    master, boot_time, waypoint[0], waypoint[1], waypoint[2]
                ),
            )

        print("Final hover ...", flush=True)
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
        else:
            print("Mission complete. Holding final waypoint in OFFBOARD.", flush=True)
    finally:
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
