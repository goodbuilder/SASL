#!/usr/bin/env python3
"""Record Gazebo GT, simulation clock, camera truth and PX4 telemetry as JSONL.

Source clocks remain separate: Gazebo exposure/GT stamps are simulation ns,
PX4 uORB stamps are microseconds, and callback receipt uses monotonic ns.
The left/right original images are recorded by capture_streams.py in lossless
rosbag2 files. This recorder stores no video and never rewrites raw rows.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import time
from pathlib import Path


STOP = False


def _stop(_signum, _frame):
    global STOP
    STOP = True


def _finite_or_none(value):
    number = float(value)
    return number if math.isfinite(number) else None


def _expected_stop_exception(exc: BaseException, stop_requested: bool) -> bool:
    if not stop_requested:
        return False
    if type(exc).__name__ in ("ExternalShutdownException", "KeyboardInterrupt"):
        return True
    return (
        type(exc).__name__ == "RCLError"
        and "failed to initialize wait set" in str(exc)
        and "context is not valid" in str(exc)
    )


class StateRecordWriter:
    FILES = {
        "clock": "clock.jsonl",
        "gt": "ground_truth.jsonl",
        "px4_local": "px4_local_position.jsonl",
        "px4_attitude": "px4_attitude.jsonl",
        "camera": "camera_pose_diagnostics.jsonl",
    }

    def __init__(self, attempt_dir: Path):
        if not attempt_dir.is_dir():
            raise ValueError("attempt directory must already exist")
        self.attempt_dir = attempt_dir
        self.handles = {}
        try:
            for name, filename in self.FILES.items():
                self.handles[name] = (attempt_dir / filename).open("x", encoding="utf-8")
        except BaseException:
            for handle in self.handles.values():
                handle.close()
            raise
        self.counts = {name: 0 for name in self.FILES}
        self.epoch = 0
        self.last_clock_ns = None
        self.first_gt_stamp_ns = None
        self.last_gt_stamp_ns = None
        self.gt_gap_ge_40ms_count = 0
        self.max_gt_gap_ns = 0
        self.camera_payloads = {}
        self.failure_reason = None

    def _write(self, stream: str, row: dict) -> None:
        self.handles[stream].write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
        self.counts[stream] += 1
        if self.counts[stream] % 50 == 0:
            self.handles[stream].flush()

    def record_clock(self, stamp_ns: int, receive_monotonic_ns: int) -> None:
        stamp = int(stamp_ns)
        if self.last_clock_ns is not None and stamp < self.last_clock_ns:
            self.epoch += 1
        self.last_clock_ns = stamp
        self._write("clock", {
            "simulation_epoch": self.epoch,
            "simulation_timestamp_ns": stamp,
            "receive_monotonic_ns": int(receive_monotonic_ns),
        })

    def record_gt(self, msg, receive_monotonic_ns: int) -> None:
        if msg.header.frame_id != "gazebo_world_enu":
            raise ValueError("GT frame differs from gazebo_world_enu")
        stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        position = [
            _finite_or_none(msg.pose.position.x),
            _finite_or_none(msg.pose.position.y),
            _finite_or_none(msg.pose.position.z),
        ]
        quaternion = [
            _finite_or_none(msg.pose.orientation.x),
            _finite_or_none(msg.pose.orientation.y),
            _finite_or_none(msg.pose.orientation.z),
            _finite_or_none(msg.pose.orientation.w),
        ]
        if None in position or None in quaternion:
            raise ValueError("GT pose contains nonfinite values")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("GT XYZW quaternion is not unit length")
        if self.last_gt_stamp_ns is not None:
            gap = stamp - self.last_gt_stamp_ns
            if gap < 0:
                raise ValueError("GT stamp decreased without a new epoch")
            self.max_gt_gap_ns = max(self.max_gt_gap_ns, gap)
            if gap >= 40_000_000:
                self.gt_gap_ge_40ms_count += 1
        if self.first_gt_stamp_ns is None:
            self.first_gt_stamp_ns = stamp
        self.last_gt_stamp_ns = stamp
        self._write("gt", {
            "simulation_epoch": self.epoch,
            "source_timestamp_ns": stamp,
            "receive_monotonic_ns": int(receive_monotonic_ns),
            "reference_link": "vehicle_reference_link",
            "position_frame": "gazebo_world_enu",
            "position_axis_order": "east,north,up",
            "position_world_enu_m": position,
            "orientation_parent_frame": "gazebo_world_enu",
            "orientation_child_frame": "vehicle_reference_link",
            "quaternion_order": "xyzw",
            "orientation_world_from_reference_xyzw": quaternion,
        })

    def record_px4_local(self, msg, receive_monotonic_ns: int) -> None:
        self._write("px4_local", {
            "simulation_epoch": self.epoch,
            "latest_clock_timestamp_ns": self.last_clock_ns,
            "px4_timestamp_us": int(msg.timestamp),
            "receive_monotonic_ns": int(receive_monotonic_ns),
            "axis_contract": "px4_local_ned:north,east,down",
            "position_local_ned_m": [_finite_or_none(msg.x), _finite_or_none(msg.y), _finite_or_none(msg.z)],
            "velocity_local_ned_mps": [_finite_or_none(msg.vx), _finite_or_none(msg.vy), _finite_or_none(msg.vz)],
            "heading_clockwise_from_north_rad": _finite_or_none(msg.heading),
            "xy_valid": bool(msg.xy_valid),
            "z_valid": bool(msg.z_valid),
            "xy_reset_counter": int(msg.xy_reset_counter),
            "z_reset_counter": int(msg.z_reset_counter),
        })

    def record_px4_attitude(self, msg, receive_monotonic_ns: int) -> None:
        self._write("px4_attitude", {
            "simulation_epoch": self.epoch,
            "latest_clock_timestamp_ns": self.last_clock_ns,
            "px4_timestamp_us": int(msg.timestamp),
            "receive_monotonic_ns": int(receive_monotonic_ns),
            "parent_frame": "px4_local_ned",
            "child_frame": "px4_body_frd",
            "quaternion_order": "wxyz",
            "quaternion_ned_from_body_wxyz": [_finite_or_none(value) for value in msg.q],
        })

    def record_camera(self, side: str, payload: str, receive_monotonic_ns: int) -> None:
        if side not in ("left", "right"):
            raise ValueError("camera side must be left or right")
        parsed = json.loads(payload)
        self.camera_payloads[side] = parsed
        self._write("camera", {
            "simulation_epoch": self.epoch,
            "camera_id": side,
            "receive_monotonic_ns": int(receive_monotonic_ns),
            "payload": parsed,
        })

    def close(self) -> dict:
        for handle in self.handles.values():
            handle.flush()
            os.fsync(handle.fileno())
            handle.close()
        duration_ns = (
            None if self.first_gt_stamp_ns is None or self.last_gt_stamp_ns is None
            else self.last_gt_stamp_ns - self.first_gt_stamp_ns
        )
        gt_hz = (
            (self.counts["gt"] - 1) * 1e9 / duration_ns
            if duration_ns and self.counts["gt"] > 1 else None
        )
        camera_truth = {
            "schema_version": "stereo_v1_1_camera_truth_v1",
            "source": "gazebo_camera_pose_diagnostics",
            "camera_payloads": self.camera_payloads,
        }
        (self.attempt_dir / "camera_truth.json").write_text(
            json.dumps(camera_truth, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        required = (self.counts["clock"] > 0 and self.counts["gt"] > 0
                    and self.counts["px4_local"] > 0 and self.counts["px4_attitude"] > 0
                    and set(self.camera_payloads) == {"left", "right"})
        status = "STATE_CAPTURE_COMPLETE" if required and self.failure_reason is None else "STATE_CAPTURE_FAILED"
        result = {
            "schema_version": "stereo_v1_1_state_capture_v1",
            "status": status,
            "failure_reason": self.failure_reason if self.failure_reason else (None if required else "required stream missing"),
            "counts": self.counts,
            "simulation_epoch": self.epoch,
            "gt_source_duration_ns": duration_ns,
            "gt_observed_hz": gt_hz,
            "gt_gap_ge_40ms_count": self.gt_gap_ge_40ms_count,
            "max_gt_gap_ns": self.max_gt_gap_ns,
        }
        with (self.attempt_dir / "state_capture_summary.json").open("x", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float)
    args = parser.parse_args()
    if args.duration_s is not None and args.duration_s < 5:
        parser.error("short capture duration must be at least 5 seconds")
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    writer = StateRecordWriter(args.attempt_dir)
    node = None
    rclpy = None
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String

        rclpy.init()
        node = Node("stereo_v1_1_state_recorder")
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)

        def guarded(action):
            global STOP
            try:
                action()
            except Exception as exc:
                writer.failure_reason = f"{type(exc).__name__}: {exc}"
                STOP = True

        node.create_subscription(Clock, "/clock", lambda msg: guarded(lambda: writer.record_clock(
            int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec), time.monotonic_ns())), qos)
        node.create_subscription(PoseStamped, "/simulation/ground_truth/vehicle_reference_link",
                                 lambda msg: guarded(lambda: writer.record_gt(msg, time.monotonic_ns())), qos)
        node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 lambda msg: guarded(lambda: writer.record_px4_local(msg, time.monotonic_ns())), qos)
        node.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                                 lambda msg: guarded(lambda: writer.record_px4_attitude(msg, time.monotonic_ns())), qos)
        for side in ("left", "right"):
            node.create_subscription(String, f"/simulation/camera_pose_diagnostics/{side}",
                                     lambda msg, eye=side: guarded(lambda: writer.record_camera(eye, msg.data, time.monotonic_ns())), qos)
        started = time.monotonic()
        ready_announced = False
        while not STOP and (args.duration_s is None or time.monotonic() - started < args.duration_s):
            rclpy.spin_once(node, timeout_sec=0.1)
            if not ready_announced and writer.counts["clock"] and writer.counts["gt"] and writer.counts["px4_local"]:
                print("STATE_READY", flush=True)
                ready_announced = True
    except BaseException as exc:
        if not _expected_stop_exception(exc, STOP):
            writer.failure_reason = writer.failure_reason or f"{type(exc).__name__}: {exc}"
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
    result = writer.close()
    print(json.dumps({"status": result["status"], "failure_reason": result["failure_reason"],
                      "counts": result["counts"], "gt_observed_hz": result["gt_observed_hz"]}, ensure_ascii=False))
    return 0 if result["status"] == "STATE_CAPTURE_COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
