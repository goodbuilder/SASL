#!/usr/bin/env python3
import argparse
import math
import sys
import threading
import time

from pymavlink import mavutil


def positive_float(value):
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


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
        "--check-connection",
        action="store_true",
        help="Connect to PX4 and print local position without arming or moving.",
    )
    return parser


class ActualDistanceMonitor:
    def __init__(self, topic="/ground_truth"):
        self.topic = topic
        self.value = None
        self._node = None
        self._thread = None
        self._rclpy = None
        self._ready = False
        self._stop = threading.Event()

    def start(self):
        try:
            import rclpy
            from std_msgs.msg import Float32
        except Exception as exc:
            print(f"Actual-distance ROS monitor unavailable: {exc}", flush=True)
            return

        self._rclpy = rclpy
        rclpy.init(args=None)
        self._node = rclpy.create_node("px4_auto_mission_distance_monitor")
        self._node.create_subscription(Float32, self.topic, self._callback, 10)
        self._ready = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _callback(self, msg):
        self.value = float(msg.data)

    def _spin(self):
        while not self._stop.is_set() and self._node is not None:
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._rclpy is not None:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass

    @property
    def ready(self):
        return self._ready


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


def wait_command_ack(master, command, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack is None:
            continue
        if int(ack.command) == int(command):
            return ack
    return None


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


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def choose_approach_vector():
    return {
        "name": "local_north_negative",
        "north_sign": -1.0,
        "east_sign": 0.0,
        "lateral_axis": "east",
    }


def approach_until_target(master, boot_time, args, x0, y0):
    monitor = ActualDistanceMonitor()
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

    print(
        f"Moving along {approach['name']} toward the cameras until actual distance <= {args.target_distance_m:.1f} m "
        f"(fallback max displacement {args.distance_m:.1f} m) ...",
        flush=True,
    )
    try:
        while time.monotonic() < deadline:
            msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            traveled = None
            lateral_error = 0.0
            if msg is not None:
                x = float(msg.x)
                y = float(msg.y)
                traveled = math.hypot(x - x0, y - y0)
                lateral_error = (x - x0) if approach["lateral_axis"] == "north" else (y - y0)

            lateral_mps = clamp(-lateral_gain * lateral_error, -max_lateral_mps, max_lateral_mps)
            north_mps = approach["north_sign"] * approach_speed_mps
            east_mps = approach["east_sign"] * approach_speed_mps
            if approach["lateral_axis"] == "north":
                north_mps += lateral_mps
            else:
                east_mps += lateral_mps
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
                    f"lateral_error={lateral_error:.1f} m, lateral={lateral_mps:.1f} m/s",
                    flush=True,
                )
                last_report = time.monotonic()
            time.sleep(period_s)
    finally:
        send_local_velocity_setpoint(master, boot_time, 0.0, 0.0, 0.0)
        monitor.stop()

    raise RuntimeError(
        f"Approach timed out before reaching {args.target_distance_m:.1f} m actual distance."
    )


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
    print("Flight authority: PX4 SITL ONLY", flush=True)
    print("PX4 Local NED: AUXILIARY ONLY — NOT GROUND TRUTH", flush=True)
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
        x0 = float(pos.x)
        y0 = float(pos.y)
        target_z = -float(args.altitude_m)
        boot_time = time.monotonic()

        # PX4 requires a short stream of setpoints before OFFBOARD mode is accepted.
        stream_for(
            2.0,
            args.setpoint_hz,
            lambda: send_position_setpoint(master, boot_time, x0, y0, target_z),
        )
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
        stream_for(
            args.first_hover_s,
            args.setpoint_hz,
            lambda: send_position_setpoint(master, boot_time, x0, y0, target_z),
        )

        approach_until_target(master, boot_time, args, x0, y0)

        print("Final hover ...", flush=True)
        stream_for(
            args.final_hover_s,
            args.setpoint_hz,
            lambda: send_body_velocity_setpoint(master, boot_time, 0.0),
        )

        print("Landing ...", flush=True)
        if "LAND" in master.mode_mapping():
            set_mode(master, "LAND")
        else:
            command_long(master, mavutil.mavlink.MAV_CMD_NAV_LAND)
        wait_landed(master, args.land_timeout_s)
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
