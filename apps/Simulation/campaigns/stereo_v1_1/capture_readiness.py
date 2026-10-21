#!/usr/bin/env python3
"""Observe stable GT after bag startup, before state capture or flight control.

This observer never arms or commands a vehicle. It gates recorder handoff using
unique Gazebo source timestamps, world ENU frame identity, simulation epoch and
host-monotonic freshness. Startup failures remain visible in raw bags; this gate
neither edits recordings nor relaxes their subsequent QC requirements.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time


class CaptureReadiness:
    def __init__(self, min_gt_hz: float, max_gt_gap_s: float,
                 stable_window_s: float = 2.0, freshness_s: float = 0.5):
        if any(not math.isfinite(v) or v <= 0 for v in
               (min_gt_hz, max_gt_gap_s, stable_window_s, freshness_s)):
            raise ValueError("readiness limits must be finite and positive")
        if stable_window_s < 2:
            raise ValueError("stable source window must be at least two simulation seconds")
        self.min_gt_hz = float(min_gt_hz)
        self.max_gap_ns = round(max_gt_gap_s * 1e9)
        self.window_ns = round(stable_window_s * 1e9)
        self.freshness_ns = round(freshness_s * 1e9)
        self.epoch = 0
        self.clock_ns = self.clock_receive_ns = self.last_receive_ns = None
        self.gt_receive_ns = None
        self.gt = deque()
        self.duplicate_gt_count = self.rejected_stale_gt_count = 0
        self.failure_reason = None

    def _receive(self, receive_ns: int):
        if self.last_receive_ns is not None and receive_ns < self.last_receive_ns:
            self.failure_reason = "host_monotonic_reception_decreased"
            raise ValueError(self.failure_reason)
        self.last_receive_ns = receive_ns

    def clock(self, stamp_ns: int, receive_ns: int):
        self._receive(receive_ns)
        if self.clock_ns is not None and stamp_ns < self.clock_ns:
            self.epoch += 1
            self.gt.clear()
            self.gt_receive_ns = None
        if self.clock_ns != stamp_ns:
            self.clock_receive_ns = receive_ns  # repeated /clock is not advancement
        self.clock_ns = stamp_ns

    def ground_truth(self, stamp_ns: int, frame_id: str, receive_ns: int):
        self._receive(receive_ns)
        if frame_id != "gazebo_world_enu":
            self.failure_reason = "ground_truth_frame_not_world_enu"
            raise ValueError(self.failure_reason)
        if (self.clock_ns is None or self.clock_receive_ns is None
                or receive_ns - self.clock_receive_ns > self.freshness_ns
                or abs(stamp_ns - self.clock_ns) > self.max_gap_ns):
            self.rejected_stale_gt_count += 1
            self.gt.clear()
            self.gt_receive_ns = None
            return
        if self.gt and stamp_ns <= self.gt[-1]:
            if stamp_ns == self.gt[-1]:
                self.duplicate_gt_count += 1
            else:
                self.gt.clear()  # backward source without clock reset: restart stability
            return
        if self.gt and stamp_ns - self.gt[-1] > self.max_gap_ns:
            self.gt.clear()
        self.gt.append(stamp_ns)
        self.gt_receive_ns = receive_ns
        # Retain the earliest sample needed for a >=2 s sliding source window.
        while len(self.gt) > 2 and self.gt[-1] - self.gt[1] >= self.window_ns:
            self.gt.popleft()

    def report(self, now_ns: int) -> dict:
        duration_ns = self.gt[-1] - self.gt[0] if len(self.gt) > 1 else 0
        rate = (len(self.gt)-1)*1e9/duration_ns if duration_ns else None
        max_gap = max((b-a for a,b in zip(self.gt, list(self.gt)[1:])), default=None)
        fresh = bool(self.clock_receive_ns is not None and self.gt_receive_ns is not None
                     and 0 <= now_ns-self.clock_receive_ns <= self.freshness_ns
                     and 0 <= now_ns-self.gt_receive_ns <= self.freshness_ns
                     and self.clock_ns is not None and self.gt
                     and abs(self.clock_ns-self.gt[-1]) <= self.max_gap_ns)
        ready = bool(not self.failure_reason and fresh and duration_ns >= self.window_ns
                     and rate is not None and rate+1e-9 >= self.min_gt_hz
                     and max_gap is not None and max_gap <= self.max_gap_ns)
        return {"schema_version": "capture_readiness_v1", "status": "READY" if ready else "WAITING",
                "failure_reason": self.failure_reason, "simulation_epoch": self.epoch,
                "source_frame": "gazebo_world_enu", "source_time_domain": "gazebo_simulation_ns",
                "receive_time_domain": "host_monotonic_ns", "unique_gt_samples": len(self.gt),
                "source_window_duration_s": duration_ns/1e9, "observed_gt_hz": rate,
                "max_gt_gap_s": max_gap/1e9 if max_gap is not None else None,
                "required_min_gt_hz": self.min_gt_hz, "allowed_max_gt_gap_s": self.max_gap_ns/1e9,
                "required_stable_window_s": self.window_ns/1e9,
                "host_freshness_limit_s": self.freshness_ns/1e9, "fresh": fresh,
                "duplicate_gt_count": self.duplicate_gt_count,
                "rejected_stale_gt_count": self.rejected_stale_gt_count}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--min-gt-hz", type=float, required=True)
    parser.add_argument("--max-gt-gap-s", type=float, required=True)
    parser.add_argument("--timeout-s", type=float, default=60)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or not 0 < args.timeout_s <= 60:
        parser.error("timeout must be positive and at most 60 host seconds")
    if not args.attempt_dir.is_dir():
        parser.error("attempt directory must exist")
    report_path = args.attempt_dir / "capture_readiness.json"
    if report_path.exists():
        raise FileExistsError("readiness evidence already exists; refusing overwrite")
    gate = CaptureReadiness(args.min_gt_hz, args.max_gt_gap_s)
    started = time.monotonic()
    node = None
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    try:
        rclpy.init()
        node = rclpy.create_node("stereo_capture_readiness")
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Clock, "/clock", lambda msg: gate.clock(
            int(msg.clock.sec)*1_000_000_000+int(msg.clock.nanosec), time.monotonic_ns()), qos)
        node.create_subscription(PoseStamped, "/simulation/ground_truth/vehicle_reference_link",
            lambda msg: gate.ground_truth(int(msg.header.stamp.sec)*1_000_000_000+
                int(msg.header.stamp.nanosec), msg.header.frame_id, time.monotonic_ns()), qos)
        while time.monotonic()-started < args.timeout_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            if gate.report(time.monotonic_ns())["status"] == "READY":
                break
        else:
            gate.failure_reason = "stable_gt_readiness_timeout"
    except (Exception, KeyboardInterrupt) as exc:
        gate.failure_reason = f"{type(exc).__name__}: {exc}"
    finally:
        report = gate.report(time.monotonic_ns())
        if report["status"] != "READY":
            report["status"] = "FAILED"
        report["duration_host_s"] = time.monotonic()-started
        with report_path.open("x", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(report), flush=True)
    return 0 if report["status"] == "READY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
