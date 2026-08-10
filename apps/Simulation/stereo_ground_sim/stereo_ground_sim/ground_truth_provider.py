"""Ground Truth timestamp routing before world-ENU-to-FRD evaluation.

Records remain world ENU positions and world-from-reference XYZW orientations.
This provider owns epoch/future/quarantine policy but performs no coordinate
conversion; accepted records are passed to the injected consumer unchanged.
"""

from collections import deque


class GroundTruthProvider:
    def __init__(self, *, capacity=64, reorder_lead_limit_ns=200_000_000):
        self.capacity = int(capacity)
        self.reorder_lead_limit_ns = int(reorder_lead_limit_ns)
        self.future = deque()
        self.quarantine_candidates = deque()
        self.quarantine_active = False
        self.quarantine_floor_stamp_ns = None
        self.quarantine_last_stamp_ns = None
        self.quarantine_required_samples = 2
        self.future_overflow_count = 0
        self.stale_count = 0
        self.quarantine_rejected_count = 0
        self.quarantine_release_count = 0

    def reject(self, *, quarantine=False, count=1):
        self.stale_count += int(count)
        if quarantine:
            self.quarantine_rejected_count += int(count)

    def route(self, record, *, epoch, latest_clock_ns, accept, warning=lambda _m: None):
        if record.epoch != epoch:
            self.reject(quarantine=True)
            return False
        floor = self.quarantine_floor_stamp_ns
        if floor is not None and record.stamp_ns <= floor:
            self.reject(quarantine=True)
            return False
        if self.quarantine_active:
            if (
                latest_clock_ns is not None
                and record.stamp_ns - latest_clock_ns > self.reorder_lead_limit_ns
            ):
                self.reject(quarantine=True)
                return False
            if (
                self.quarantine_last_stamp_ns is not None
                and record.stamp_ns <= self.quarantine_last_stamp_ns
            ):
                self.reject(quarantine=True, count=len(self.quarantine_candidates))
                self.quarantine_candidates.clear()
            if len(self.quarantine_candidates) >= self.capacity:
                self.quarantine_candidates.popleft()
                self.future_overflow_count += 1
            self.quarantine_candidates.append(record)
            self.quarantine_last_stamp_ns = record.stamp_ns
            self.release_if_ready(
                epoch=epoch, latest_clock_ns=latest_clock_ns, accept=accept, warning=warning
            )
            return False
        return self.queue_or_accept(
            record, epoch=epoch, latest_clock_ns=latest_clock_ns,
            accept=accept, warning=warning,
        )

    def release_if_ready(self, *, epoch, latest_clock_ns, accept, warning=lambda _m: None):
        if not self.quarantine_active:
            return False
        if (
            latest_clock_ns is None
            or self.quarantine_floor_stamp_ns is None
            or latest_clock_ns < self.quarantine_floor_stamp_ns
            or len(self.quarantine_candidates) < self.quarantine_required_samples
        ):
            return False
        candidates = tuple(self.quarantine_candidates)
        self.quarantine_active = False
        self.quarantine_release_count += 1
        self.quarantine_candidates.clear()
        self.quarantine_last_stamp_ns = None
        for record in candidates:
            self.queue_or_accept(
                record, epoch=epoch, latest_clock_ns=latest_clock_ns,
                accept=accept, warning=warning,
            )
        return True

    def queue_or_accept(self, record, *, epoch, latest_clock_ns, accept, warning=lambda _m: None):
        if latest_clock_ns is not None and record.stamp_ns > latest_clock_ns:
            lead_ns = record.stamp_ns - latest_clock_ns
            if lead_ns > self.reorder_lead_limit_ns:
                self.reject()
                warning(
                    "Ground Truth future stamp rejected as stale pre-reset data: "
                    f"stamp={record.stamp_ns}, clock={latest_clock_ns}, lead_ns={lead_ns}"
                )
                return False
            if len(self.future) >= self.capacity:
                self.future.popleft()
                self.future_overflow_count += 1
            self.future.append(record)
            return False
        return accept(record)

    def drain(self, clock_stamp_ns, *, epoch, accept):
        self.release_if_ready(
            epoch=epoch, latest_clock_ns=clock_stamp_ns, accept=accept
        )
        ready, pending = [], deque()
        while self.future:
            record = self.future.popleft()
            if record.epoch != epoch:
                self.reject(quarantine=True)
            elif record.stamp_ns <= clock_stamp_ns:
                ready.append(record)
            else:
                pending.append(record)
        self.future = pending
        for record in sorted(ready, key=lambda item: item.stamp_ns):
            accept(record)

    def reset(self, reset_stamp_ns, quarantine_duration_ns):
        self.stale_count += len(self.future) + len(self.quarantine_candidates)
        self.future.clear()
        self.quarantine_candidates.clear()
        self.quarantine_last_stamp_ns = None
        self.quarantine_active = True
        self.quarantine_floor_stamp_ns = int(reset_stamp_ns) + int(quarantine_duration_ns)
