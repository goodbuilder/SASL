"""Dispatch science references from observed /clock callbacks, never wall ticks.

Snapshot position remains world ENU / PX4 local NED as supplied by its owner.
This module owns timing only. A callback is an observed source-clock event, not
proof of PX4 reception/actuation. No synthetic or historical catch-up is sent.
"""
from __future__ import annotations

import math
import threading
import time


class ScienceDispatcher:
    """Science timing owner with optional buffered, nonblocking diagnostic sink.

    Source timestamps are simulation nanoseconds; receipt/lock/end timestamps
    are host monotonic nanoseconds. Sink failures are counted, not propagated.
    """
    def __init__(self, start_snapshot, duration_s, hz, send_update, send_keepalive,
                 start_host_ns, diagnostic_sink=None):
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError('hz must be positive and finite')
        self.diagnostic_sink = diagnostic_sink
        self.diagnostic_error_count = 0
        self.lock = threading.RLock()
        self.start_ns = start_snapshot.simulation_time_ns
        self.end_ns = self.start_ns + round(duration_s * 1e9)
        self.period_ns = round(1e9 / hz)
        self.epoch = start_snapshot.simulation_epoch
        self.resets = (start_snapshot.xy_reset_counter, start_snapshot.z_reset_counter)
        self.last_stamp_ns = None
        self.last_update_ns = None
        self.last_update_slot = -1
        self.last_send_host_ns = start_host_ns
        self.last_valid_host_ns = start_host_ns
        self.last_snapshot = start_snapshot
        self.finished = False
        self.send_update = send_update
        self.send_keepalive = send_keepalive

    def progress_snapshot(self):
        """Atomic readback after callback completion, for source-step admission."""
        with self.lock:
            return {"observed_timestamp_ns": self.last_stamp_ns,
                    "last_reference_timestamp_ns": self.last_update_ns,
                    "finished": self.finished,
                    "snapshot": self.last_snapshot}

    def _diagnosed_call(self, action, operation, snapshot=None, received_host_ns=None):
        if self.diagnostic_sink is None:
            return operation()
        entered = time.monotonic_ns()
        details = {"component": "science_dispatch", "operation": action,
                   "action": "error", "dispatch_enter_monotonic_ns": entered,
                   "clock_receive_monotonic_ns": received_host_ns,
                   "source_timestamp_ns": None if snapshot is None else snapshot.simulation_time_ns,
                   "slot": None if snapshot is None else
                       (snapshot.simulation_time_ns - self.start_ns) // self.period_ns}
        try:
            with self.lock:
                acquired = time.monotonic_ns()
                details.update(dispatch_lock_acquired_monotonic_ns=acquired,
                               dispatch_lock_wait_ns=acquired-entered)
                details["action"] = operation()
        except BaseException as exc:
            details["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            details["dispatch_end_monotonic_ns"] = time.monotonic_ns()
            try:
                self.diagnostic_sink(details)
            except Exception:
                self.diagnostic_error_count += 1

    def on_clock(self, snapshot, received_host_ns):
        return self._diagnosed_call("clock", lambda: self._on_clock(snapshot, received_host_ns),
                                    snapshot, received_host_ns)

    def _on_clock(self, snapshot, received_host_ns):
        with self.lock:
            if self.finished:
                return "finished"
            if snapshot is None:
                return "snapshot_rejected"
            stamp = snapshot.simulation_time_ns
            if snapshot.simulation_epoch != self.epoch:
                raise RuntimeError('simulation clock reset during science')
            if (snapshot.xy_reset_counter, snapshot.z_reset_counter) != self.resets:
                raise RuntimeError('PX4 local frame reset during science')
            if stamp < self.start_ns or (self.last_stamp_ns is not None and stamp < self.last_stamp_ns):
                raise RuntimeError('science simulation time moved backwards')
            self.last_stamp_ns = stamp
            self.last_snapshot = snapshot
            self.last_valid_host_ns = received_host_ns
            if stamp >= self.end_ns:
                self.finished = True
                return "science_end"
            slot = (stamp - self.start_ns) // self.period_ns
            if slot <= self.last_update_slot:
                return "within_slot"
            self.send_update(snapshot, (stamp - self.start_ns) / 1e9, received_host_ns)
            self.last_update_ns = stamp
            self.last_update_slot = slot
            self.last_send_host_ns = received_host_ns
            return "reference_update"

    def watchdog(self, host_ns):
        return self._diagnosed_call("watchdog", lambda: self._watchdog(host_ns))

    def _watchdog(self, host_ns):
        with self.lock:
            if self.finished:
                return "finished"
            if host_ns - self.last_valid_host_ns > 1_000_000_000:
                raise RuntimeError('science telemetry became stale')
            if host_ns - self.last_send_host_ns >= self.period_ns:
                # Host keepalive maintains offboard availability during a paused
                # clock; it is explicitly not a new simulation reference update.
                self.send_keepalive()
                self.last_send_host_ns = host_ns
                return "keepalive"
            return "not_due"
