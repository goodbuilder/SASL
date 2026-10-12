#!/usr/bin/env python3
"""Follow one v1.1 time-reference flight through PX4 SITL offboard control.

This is a campaign component, not a 39-run launcher or a completed capture.
It requires a separately running Gazebo/PX4/ROS stack and an attempt directory.
The caller must start lossless raw recording before invoking this controller.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from pathlib import Path

from pymavlink import mavutil


SIM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_ROOT))
sys.path.insert(0, str(SIM_ROOT / "scripts"))

from px4_auto_mission import (
    GcsHeartbeat, arm, now_ms, request_land, set_mode,
    wait_heartbeat, wait_landed,
)

from campaigns.stereo_v1_1.campaign_setpoints import (
    send_position_velocity_yaw, send_position_yaw,
)
from campaigns.stereo_v1_1.flight_gates import (
    OriginObservation, SettleSample, fit_observed_origin, science_start_ready,
)
from campaigns.stereo_v1_1.ros_observation_bridge import RosObservationBridge
from campaigns.stereo_v1_1.runtime_observation import FlightSnapshot, ObservationCache
from campaigns.stereo_v1_1.trajectory_contract import (
    LocalNed, LocalNedVelocity, WorldEnu,
    heading_clockwise_from_north_to_px4_yaw_rad, interpolate_reference,
    world_position_to_px4_local_ned, world_velocity_to_px4_local_ned,
)


STOP_REQUESTED = threading.Event()


def _stop_signal(_signum, _frame):
    STOP_REQUESTED.set()


def _check_stop():
    if STOP_REQUESTED.is_set():
        raise KeyboardInterrupt("flight controller was interrupted")


def _heading_error_deg(actual_rad: float, target_rad: float) -> float:
    delta = math.atan2(math.sin(actual_rad - target_rad), math.cos(actual_rad - target_rad))
    return abs(math.degrees(delta))


def _quaternion_yaw_deg_xyzw(values) -> float:
    x, y, z, w = (float(value) for value in values)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _angle_error_deg(actual_deg: float, expected_deg: float) -> float:
    delta = math.radians(actual_deg - expected_deg)
    return abs(math.degrees(math.atan2(math.sin(delta), math.cos(delta))))


def bounded_approach_target(actual: LocalNed, goal: LocalNed, max_lead_m: float = 5.0) -> LocalNed:
    """Keep the PX4 position setpoint within max_lead_m of current local pose."""
    if not math.isfinite(max_lead_m) or max_lead_m <= 0:
        raise ValueError("max_lead_m must be positive and finite")
    delta = tuple(goal_value - actual_value for goal_value, actual_value in zip(goal.as_xyz(), actual.as_xyz()))
    distance = math.sqrt(sum(value * value for value in delta))
    if distance <= max_lead_m:
        return goal
    scale = max_lead_m / distance
    return LocalNed(*(actual.as_xyz()[axis] + scale * delta[axis] for axis in range(3)))


class AttemptLog:
    def __init__(self, attempt_dir: Path, run_id: str, attempt_id: str) -> None:
        self.run_id = run_id
        self.attempt_id = attempt_id
        self.events = (attempt_dir / "events.jsonl").open("x", encoding="utf-8")
        self.states = (attempt_dir / "flight_state.jsonl").open("x", encoding="utf-8")
        self.result_path = attempt_dir / "controller_result.json"
        self.coordinate_check_path = attempt_dir / "coordinate_frame_check.json"

    def event(self, name: str, snapshot: FlightSnapshot | None = None, **details) -> None:
        row = {
            "run_id": self.run_id, "attempt_id": self.attempt_id,
            "event": name, "utc_ns": time.time_ns(),
            "simulation_epoch": None if snapshot is None else snapshot.simulation_epoch,
            "simulation_timestamp_ns": None if snapshot is None else snapshot.simulation_time_ns,
            "details": details,
        }
        self.events.write(json.dumps(row, allow_nan=False, sort_keys=True) + "\n")
        self.events.flush()

    def state(
        self, phase: str, snapshot: FlightSnapshot,
        target_position: LocalNed | None,
        target_velocity: LocalNedVelocity | None,
        origin: WorldEnu | None,
        reference: dict | None = None,
    ) -> None:
        row = {
            "run_id": self.run_id, "attempt_id": self.attempt_id,
            "phase": phase, "simulation_epoch": snapshot.simulation_epoch,
            "simulation_timestamp_ns": snapshot.simulation_time_ns,
            "gt_timestamp_ns": snapshot.gt_stamp_ns,
            "gt_reference_position_world_enu_m": snapshot.gt_world_enu_m.as_xyz(),
            "px4_local_position_ned_m": snapshot.px4_local_ned_m.as_xyz(),
            "px4_local_velocity_ned_mps": snapshot.px4_velocity_ned_mps.as_xyz(),
            "px4_heading_rad": snapshot.px4_heading_rad,
            "px4_timestamp_us": snapshot.px4_timestamp_us,
            "px4_xy_reset_counter": snapshot.xy_reset_counter,
            "px4_z_reset_counter": snapshot.z_reset_counter,
            "origin_world_enu_m": None if origin is None else origin.as_xyz(),
            "setpoint_position_local_ned_m": None if target_position is None else target_position.as_xyz(),
            "setpoint_velocity_local_ned_mps": None if target_velocity is None else target_velocity.as_xyz(),
            "reference_position_world_enu_m": None if reference is None else reference["position_world_enu_m"],
            "reference_velocity_world_enu_mps": None if reference is None else reference["velocity_world_enu_mps"],
        }
        self.states.write(json.dumps(row, allow_nan=False, sort_keys=True) + "\n")

    def finish(self, status: str, **details) -> None:
        self.events.flush()
        self.states.flush()
        os.fsync(self.events.fileno())
        os.fsync(self.states.fileno())
        self.events.close()
        self.states.close()
        with self.result_path.open("x", encoding="utf-8") as handle:
            json.dump({
                "run_id": self.run_id, "attempt_id": self.attempt_id,
                "controller_status": status, "utc_ns": time.time_ns(), **details,
            }, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())

    def coordinate_frame_check(self, **details) -> None:
        if self.coordinate_check_path.exists():
            raise FileExistsError("coordinate frame check was already written")
        with self.coordinate_check_path.open("x", encoding="utf-8") as handle:
            json.dump({"status": "PASS", "schema_version": "coordinate_frame_check_v1",
                       "method": "runtime origin fit plus independent world/local science-start gates",
                       **details}, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())


def _snapshot(bridge: RosObservationBridge, cache: ObservationCache) -> FlightSnapshot | None:
    if bridge.last_error:
        raise RuntimeError(f"ROS observation failed: {bridge.last_error}")
    return cache.snapshot(time.monotonic_ns())


def _collect_origin(bridge, cache, log, frame_gate: dict | None = None,
                    vehicle: dict | None = None) -> tuple[WorldEnu, FlightSnapshot, dict]:
    frame_gate = frame_gate or {}
    vehicle = vehicle or {}
    timeout_s = float(frame_gate.get("origin_timeout_s", 30.0))
    min_samples = int(frame_gate.get("origin_min_samples", 20))
    max_speed = float(frame_gate.get("origin_max_speed_mps", 0.3))
    max_skew_ns = int(float(frame_gate.get("origin_max_receive_skew_s", 0.1)) * 1_000_000_000)
    max_residual = float(frame_gate.get("origin_max_residual_m", 0.5))
    deadline = time.monotonic() + timeout_s
    observations: list[OriginObservation] = []
    last_identity = None
    final = None
    while time.monotonic() < deadline:
        _check_stop()
        snap = _snapshot(bridge, cache)
        if snap is not None:
            identity = (snap.gt_receive_monotonic_ns, snap.px4_receive_monotonic_ns)
            if identity != last_identity and snap.speed_mps < max_speed:
                observations.append(OriginObservation(
                    snap.gt_world_enu_m, snap.px4_local_ned_m,
                    snap.gt_receive_monotonic_ns, snap.px4_receive_monotonic_ns,
                    snap.simulation_epoch, snap.xy_reset_counter, snap.z_reset_counter,
                ))
                last_identity, final = identity, snap
                # Keep only one reset generation. A fresh sampler will retry.
                if len(observations) >= min_samples:
                    try:
                        origin = fit_observed_origin(
                            observations[-min_samples:], min_samples=min_samples,
                            max_receive_skew_ns=max_skew_ns, max_residual_m=max_residual,
                        )
                    except ValueError:
                        observations.clear()
                    else:
                        log.event("px4_local_origin_observed", snap,
                                  origin_world_enu_m=origin.as_xyz(), sample_count=min_samples,
                                  max_receive_skew_s=max_skew_ns / 1_000_000_000,
                                  max_residual_m=max_residual)
                        requested = bool(vehicle.get("verify_spawn_yaw", 0))
                        if requested:
                            actual_yaw = _quaternion_yaw_deg_xyzw(
                                snap.gt_orientation_world_from_reference_xyzw
                            )
                            expected_yaw = float(vehicle["spawn_yaw_deg"])
                            yaw_error = _angle_error_deg(actual_yaw, expected_yaw)
                            spawn_check = {
                                "status": "PASS" if yaw_error <= 2.0 else "FAIL",
                                "requested": True,
                                "simulator_spawn_yaw_deg": expected_yaw,
                                "observed_world_enu_yaw_deg": actual_yaw,
                                "absolute_error_deg": yaw_error,
                                "tolerance_deg": 2.0,
                            }
                            log.event("vehicle_spawn_yaw_checked", snap, **spawn_check)
                            if spawn_check["status"] != "PASS":
                                raise RuntimeError(
                                    f"Gazebo spawn yaw differs from the observed world ENU yaw by {yaw_error:.3f} degrees"
                                )
                        else:
                            spawn_check = {"status": "NOT_REQUESTED", "requested": False}
                        return origin, snap, spawn_check
        time.sleep(0.05)
    raise RuntimeError(
        f"could not observe a stable PX4 local origin within {timeout_s:g} seconds"
    )


def _tick_sleep(next_wall_s: float) -> float:
    remaining = next_wall_s - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)
    return next_wall_s


def _fly(args, samples, log: AttemptLog, run_config: dict | None = None) -> dict:
    run_config = run_config or {}
    frame_gate = run_config.get("FrameGate", {})
    entry = run_config.get("Entry", {})
    motion = run_config.get("Motion", {})
    vehicle = run_config.get("Vehicle", {})
    args.setpoint_hz = float(motion.get("setpoint_hz", args.setpoint_hz))
    args.entry_timeout_s = float(entry.get("entry_timeout_s", args.entry_timeout_s))
    args.entry_lead_m = float(entry.get("entry_lead_m", args.entry_lead_m))
    args.land_timeout_s = float(entry.get("landing_timeout_s", args.land_timeout_s))
    cache = ObservationCache()
    bridge = RosObservationBridge(cache)
    master = None
    heartbeat = None
    offboard_attempted = False
    science_started = False
    last_snapshot = None
    origin = None
    period = 1.0 / args.setpoint_hz
    boot_time = time.monotonic()
    try:
        master = mavutil.mavlink_connection(args.connect, autoreconnect=True)
        wait_heartbeat(master, args.heartbeat_timeout_s)
        heartbeat = GcsHeartbeat(master)
        heartbeat.start()
        origin, last_snapshot, spawn_yaw_check = _collect_origin(
            bridge, cache, log, frame_gate, vehicle,
        )
        if args.preflight_only:
            log.event("controller_preflight_complete", last_snapshot,
                      origin_world_enu_m=origin.as_xyz())
            return {"controller_status": "preflight_complete",
                    "origin_world_enu_m": origin.as_xyz()}
        first = samples[0]
        start_world = WorldEnu.from_reference_xyz(first["position_world_enu_m"])
        goal = world_position_to_px4_local_ned(start_world, origin)
        yaw = heading_clockwise_from_north_to_px4_yaw_rad(first["heading_deg_clockwise_from_north"])
        hold_at = last_snapshot.px4_local_ned_m

        # PX4 requires streamed offboard setpoints before mode and arm.
        for phase in ("prestream", "mode_confirm"):
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                _check_stop()
                send_position_yaw(master, now_ms(boot_time), hold_at, yaw)
                time.sleep(period)
            if phase == "prestream":
                set_mode(master, "OFFBOARD")
                offboard_attempted = True
        arm(master)
        log.event("entry_started", last_snapshot, target_position_world_enu_m=start_world.as_xyz())

        entry_deadline = time.monotonic() + args.entry_timeout_s
        settle_deadline = None
        settle_duration_s = float(entry.get("science_start_stable_s", 5.0))
        stable_samples: deque[SettleSample] = deque(
            maxlen=max(500, int(args.setpoint_hz * (settle_duration_s + 5.0)))
        )
        last_reset = (last_snapshot.xy_reset_counter, last_snapshot.z_reset_counter)
        epoch = last_snapshot.simulation_epoch
        last_fresh_wall = time.monotonic()
        next_tick = time.monotonic()
        while time.monotonic() < entry_deadline:
            _check_stop()
            snap = _snapshot(bridge, cache)
            if snap is None:
                if time.monotonic() - last_fresh_wall > 1.0:
                    raise RuntimeError("entry telemetry became stale")
                send_position_yaw(master, now_ms(boot_time), hold_at, yaw)
                next_tick = _tick_sleep(next_tick + period)
                continue
            last_fresh_wall, last_snapshot = time.monotonic(), snap
            if snap.simulation_epoch != epoch:
                raise RuntimeError("simulation clock reset during entry")
            reset = (snap.xy_reset_counter, snap.z_reset_counter)
            if reset != last_reset:
                log.event("px4_local_reset_during_entry", snap, old=last_reset, new=reset)
                # Stop advancing and observe the new coordinate origin while hovering.
                hold_at = snap.px4_local_ned_m
                for _ in range(int(args.setpoint_hz * 2)):
                    _check_stop()
                    send_position_yaw(master, now_ms(boot_time), hold_at, yaw)
                    time.sleep(period)
                origin, last_snapshot, spawn_yaw_check = _collect_origin(
                    bridge, cache, log, frame_gate, vehicle,
                )
                goal = world_position_to_px4_local_ned(start_world, origin)
                last_reset = (last_snapshot.xy_reset_counter, last_snapshot.z_reset_counter)
                stable_samples.clear()
                continue
            actual = snap.px4_local_ned_m
            target = bounded_approach_target(actual, goal, args.entry_lead_m)
            send_position_yaw(master, now_ms(boot_time), target, yaw)
            hold_at = target
            error_m = math.dist(actual.as_xyz(), goal.as_xyz())
            world_error_m = math.dist(snap.gt_world_enu_m.as_xyz(), start_world.as_xyz())
            heading_error = _heading_error_deg(snap.px4_heading_rad, yaw)
            if not stable_samples or snap.simulation_time_ns > stable_samples[-1].simulation_time_ns:
                stable_samples.append(SettleSample(
                    snap.simulation_time_ns, snap.speed_mps, error_m, heading_error,
                    snap.simulation_epoch, snap.xy_reset_counter, snap.z_reset_counter,
                    world_error_m,
                ))
            log.state("entry", snap, target, None, origin)
            start_window_m = float(entry.get("science_start_window_m", 1.0))
            readiness_timeout_s = float(entry.get("science_start_readiness_timeout_s", 120.0))
            if error_m < start_window_m and settle_deadline is None:
                settle_deadline = time.monotonic() + readiness_timeout_s
                log.event("settle_window_entered", snap, position_error_m=error_m,
                          start_window_m=start_window_m,
                          readiness_timeout_s=readiness_timeout_s)
            if settle_deadline is not None and time.monotonic() > settle_deadline:
                raise RuntimeError(
                    f"science readiness not achieved within {readiness_timeout_s:g} seconds"
                )
            if science_start_ready(
                list(stable_samples),
                duration_ns=round(float(entry.get("science_start_stable_s", 5.0)) * 1_000_000_000),
                max_sample_gap_ns=round(float(entry.get("science_start_max_gap_s", 0.2)) * 1_000_000_000),
                max_speed_mps=float(entry.get("science_start_speed_mps", 0.3)),
                max_position_error_m=float(entry.get("science_start_position_error_m", 0.5)),
                max_heading_error_deg=float(entry.get("science_start_heading_error_deg", 3.0)),
            ):
                world_error_m = math.dist(snap.gt_world_enu_m.as_xyz(), start_world.as_xyz())
                log.coordinate_frame_check(
                    run_id=log.run_id,
                    attempt_id=log.attempt_id,
                    simulation_epoch=snap.simulation_epoch,
                    observed_origin_world_enu_m=origin.as_xyz(),
                    reference_start_world_enu_m=start_world.as_xyz(),
                    observed_vehicle_world_enu_m=snap.gt_world_enu_m.as_xyz(),
                    expected_px4_local_ned_m=goal.as_xyz(),
                    observed_px4_local_ned_m=snap.px4_local_ned_m.as_xyz(),
                    local_start_position_error_m=math.dist(snap.px4_local_ned_m.as_xyz(), goal.as_xyz()),
                    world_start_position_error_m=world_error_m,
                    speed_mps=snap.speed_mps,
                    heading_error_deg=_heading_error_deg(snap.px4_heading_rad, yaw),
                    origin_xy_reset_counter=snap.xy_reset_counter,
                    origin_z_reset_counter=snap.z_reset_counter,
                    origin_observations_required=int(frame_gate.get("origin_min_samples", 20)),
                    same_simulation_epoch_required=bool(frame_gate.get("origin_same_epoch", 1)),
                    stable_reset_counters_required=bool(frame_gate.get("origin_stable_reset", 1)),
                    verify_enu_ned_axes_requested=bool(frame_gate.get("verify_enu_ned_axes", 1)),
                    vehicle_spawn_yaw_check=spawn_yaw_check,
                )
                log.event("science_start", snap, origin_world_enu_m=origin.as_xyz(),
                          world_position_error_m=world_error_m,
                          local_position_error_m=math.dist(snap.px4_local_ned_m.as_xyz(), goal.as_xyz()))
                science_started = True
                break
            next_tick = _tick_sleep(next_tick + period)
        else:
            raise RuntimeError("entry trajectory did not reach the science start")

        science_start_ns = last_snapshot.simulation_time_ns
        duration_s = float(samples[-1]["t_science_s"])
        next_tick = time.monotonic()
        last_fresh_wall = time.monotonic()
        last_science_position = goal
        last_science_velocity = LocalNedVelocity(0.0, 0.0, 0.0)
        while True:
            _check_stop()
            snap = _snapshot(bridge, cache)
            if snap is None:
                if time.monotonic() - last_fresh_wall > 1.0:
                    raise RuntimeError("science telemetry became stale")
                send_position_velocity_yaw(
                    master, now_ms(boot_time), last_science_position,
                    last_science_velocity, yaw,
                )
                next_tick = _tick_sleep(next_tick + period)
                continue
            last_fresh_wall, last_snapshot = time.monotonic(), snap
            if snap.simulation_epoch != epoch:
                raise RuntimeError("simulation clock reset during science")
            if (snap.xy_reset_counter, snap.z_reset_counter) != last_reset:
                raise RuntimeError("PX4 local frame reset during science")
            elapsed = (snap.simulation_time_ns - science_start_ns) / 1_000_000_000.0
            if elapsed < 0:
                raise RuntimeError("science simulation time moved backwards")
            if elapsed >= duration_s:
                break
            reference = interpolate_reference(samples, elapsed)
            world = WorldEnu.from_reference_xyz(reference["position_world_enu_m"])
            local = world_position_to_px4_local_ned(world, origin)
            velocity = world_velocity_to_px4_local_ned(reference["velocity_world_enu_mps"])
            send_position_velocity_yaw(master, now_ms(boot_time), local, velocity, yaw)
            last_science_position, last_science_velocity = local, velocity
            log.state("science", snap, local, velocity, origin, reference)
            next_tick = _tick_sleep(next_tick + period)

        final_world = WorldEnu.from_reference_xyz(samples[-1]["position_world_enu_m"])
        final_local = world_position_to_px4_local_ned(final_world, origin)
        send_position_yaw(master, now_ms(boot_time), final_local, yaw)
        log.event("science_end", last_snapshot, duration_reference_s=duration_s)
        hold_start_ns = last_snapshot.simulation_time_ns
        next_tick = time.monotonic()
        last_fresh_wall = time.monotonic()
        while True:
            _check_stop()
            snap = _snapshot(bridge, cache)
            if snap is None:
                if time.monotonic() - last_fresh_wall > 1.0:
                    raise RuntimeError("post-science telemetry became stale")
                send_position_yaw(master, now_ms(boot_time), final_local, yaw)
                next_tick = _tick_sleep(next_tick + period)
                continue
            last_fresh_wall = time.monotonic()
            last_snapshot = snap
            if snap.simulation_epoch != epoch:
                raise RuntimeError("simulation clock reset during post-science hold")
            postscience_hold_s = float(entry.get("postscience_hold_s", 5.0))
            if snap.simulation_time_ns - hold_start_ns >= round(postscience_hold_s * 1_000_000_000):
                break
            send_position_yaw(master, now_ms(boot_time), final_local, yaw)
            log.state("postscience_hold", snap, final_local, None, origin)
            next_tick = _tick_sleep(next_tick + period)
        log.event("postscience_hold_end", last_snapshot)
        request_land(master)
        log.event("landing_requested", last_snapshot)
        wait_landed(master, args.land_timeout_s)
        log.event("landing_confirmed", last_snapshot)
        return {"controller_status": "science_completed", "science_duration_reference_s": duration_s}
    except BaseException:
        if offboard_attempted and master is not None:
            try:
                request_land(master)
                log.event("emergency_landing_requested", last_snapshot,
                          science_started=science_started)
            except Exception as exc:
                log.event("emergency_landing_request_failed", last_snapshot, reason=str(exc))
        raise
    finally:
        if heartbeat is not None:
            heartbeat.stop()
        if master is not None:
            master.close()
        bridge.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--attempt-id", required=True)
    parser.add_argument("--reference-json", type=Path, required=True)
    parser.add_argument("--run-config-json", type=Path,
                        help="workbook parameters frozen for this attempt")
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--connect", default="udpin:0.0.0.0:14540")
    parser.add_argument("--preflight-only", action="store_true",
                        help="Observe PX4/Gazebo origin and exit without mode, arm or flight commands")
    parser.add_argument("--setpoint-hz", type=float, default=20.0)
    parser.add_argument("--heartbeat-timeout-s", type=float, default=30.0)
    parser.add_argument("--entry-timeout-s", type=float, default=300.0)
    parser.add_argument("--entry-lead-m", type=float, default=5.0)
    parser.add_argument("--land-timeout-s", type=float, default=90.0)
    args = parser.parse_args()
    if args.setpoint_hz < 10 or args.setpoint_hz > 100:
        parser.error("--setpoint-hz must be between 10 and 100")
    if args.entry_timeout_s <= 0 or args.entry_lead_m <= 0 or args.land_timeout_s <= 0:
        parser.error("timeouts and entry lead must be positive")
    reference = json.loads(args.reference_json.read_text(encoding="utf-8"))
    run_config = json.loads(args.run_config_json.read_text(encoding="utf-8")) if args.run_config_json else None
    samples = reference["samples"]
    if not samples or samples[0]["t_science_s"] != 0:
        parser.error("reference must start at science time zero")
    if args.attempt_dir.exists() is False:
        parser.error("attempt directory must already exist")
    logger = AttemptLog(args.attempt_dir, args.run_id, args.attempt_id)
    signal.signal(signal.SIGINT, _stop_signal)
    signal.signal(signal.SIGTERM, _stop_signal)
    try:
        result = _fly(args, samples, logger, run_config)
    except BaseException as exc:
        logger.event("controller_failed", reason=f"{type(exc).__name__}: {exc}")
        logger.finish("failed", failure_type=type(exc).__name__, failure_reason=str(exc))
        raise
    logger.finish(result["controller_status"],
                  **{k: v for k, v in result.items() if k != "controller_status"})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
