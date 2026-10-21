"""Bounded diagnostic evidence; callbacks append in memory, shutdown persists.

All *_monotonic_ns fields use the host monotonic clock. Source timestamps use
ROS simulation nanoseconds, never PX4 boot time. No coordinate values are moved.
This diagnostic can perturb scheduling; it cannot certify an uninstrumented run.
"""
from __future__ import annotations

import gc
import time
from contextlib import contextmanager
import json
import os
import threading
from collections import deque
from pathlib import Path


class BufferedTimingDiagnostics:
    def __init__(self, capacity=500_000):
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.capacity = capacity
        self.rows = deque()
        self.dropped = 0
        self.lock = threading.RLock()
        self.diagnostic_error_count = 0
        self._gc_hook = None

    def __call__(self, row):
        with self.lock:
            if len(self.rows) >= self.capacity:
                self.dropped += 1
            else:
                self.rows.append(row)

    def start_gc_tracking(self):
        """Observe Python collections without changing thresholds/enabled state."""
        if self._gc_hook is None:
            self._gc_hook = self._observe_gc
            gc.callbacks.append(self._gc_hook)

    def stop_gc_tracking(self):
        if self._gc_hook is not None:
            gc.callbacks.remove(self._gc_hook)
            self._gc_hook = None

    def _observe_gc(self, phase, info):
        try:
            self({"component": "python_gc", "action": phase,
                  "host_monotonic_ns": time.monotonic_ns(),
                  "thread_cpu_ns": time.thread_time_ns(),
                  "thread_id": threading.get_ident(),
                  "generation": info.get("generation"),
                  "collected": info.get("collected"),
                  "uncollectable": info.get("uncollectable")})
        except Exception:
            self.diagnostic_error_count += 1

    def persist(self, attempt_dir: Path):
        self.stop_gc_tracking()
        # Caller must first stop/join all producers. Exclusive creation preserves
        # previous evidence and makes accidental double-finalization visible.
        path = attempt_dir / "controller_timing_diagnostics.jsonl"
        with path.open("x", encoding="utf-8") as handle:
            for row in self.rows:
                handle.write(json.dumps({"schema_version": "controller_timing_diagnostic_v1",
                                         **row}, allow_nan=False) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        producer_errors = self.diagnostic_error_count + sum(row.get("bridge_error_count", 0) + row.get("dispatcher_error_count", 0)
                              for row in self.rows if row.get("component") == "diagnostic_producers")
        producer_stop = [row for row in self.rows if row.get("component") == "diagnostic_producers"]
        shutdown_verified = bool(producer_stop) and all(
            row.get("observer_shutdown_verified") is True for row in producer_stop)
        summary = {"schema_version": "controller_timing_diagnostic_summary_v2",
                   "method": "buffered_controller_callback_trace",
                   "file": path.name, "record_count": len(self.rows),
                   "dropped_record_count": self.dropped, "capacity": self.capacity,
                   "producer_error_count": producer_errors,
                   "producer_shutdown_verified": shutdown_verified,
                   "record_accounting_complete": self.dropped == 0 and producer_errors == 0,
                   "complete": self.dropped == 0 and producer_errors == 0 and shutdown_verified,
                   "scope": "timing diagnosis only; execution role is in run_manifest; no campaign acceptance override",
                   "clock_contract": "host monotonic ns; ROS simulation source ns"}
        with (attempt_dir / "controller_timing_diagnostics_summary.json").open("x", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        return summary


@contextmanager
def timed_stage(sink, stage: str, source_timestamp_ns: int):
    """Record wall and current-thread CPU duration, without synchronous I/O."""
    if sink is None:
        yield
        return
    wall_start = time.monotonic_ns()
    cpu_start = time.thread_time_ns()
    error = None
    try:
        yield
    except BaseException as exc:
        error = type(exc).__name__
        raise
    finally:
        cpu_end = time.thread_time_ns()
        wall_end = time.monotonic_ns()
        sink({"component": "reference_update_stage", "action": stage,
              "source_timestamp_ns": source_timestamp_ns,
              "thread_id": threading.get_ident(),
              "start_monotonic_ns": wall_start, "end_monotonic_ns": wall_end,
              "start_thread_cpu_ns": cpu_start, "end_thread_cpu_ns": cpu_end,
              "wall_duration_ns": wall_end-wall_start,
              "thread_cpu_duration_ns": cpu_end-cpu_start,
              "error_type": error})
