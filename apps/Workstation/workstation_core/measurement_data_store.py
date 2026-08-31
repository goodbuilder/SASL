"""Availability-first append storage for Workstation measurement data.

The store deliberately owns no Measurement or Session lifecycle.  Producers
enqueue independently timestamped records and continue even when a stream is
unwritable or overloaded.  One background thread is the only writer for the
fixed type files below.
"""

from __future__ import annotations

import json
import os
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Mapping
from collections.abc import Iterator, Sequence


DATA_STORE_SCHEMA = "workstation-measurement-data-store-v1"
DATA_RECORD_SCHEMA = "workstation-measurement-data-record-v1"
STORE_LIFECYCLE_OPEN = "OPEN"
STORE_LIFECYCLE_CLOSING = "CLOSING"
STORE_LIFECYCLE_CLOSED = "CLOSED"
ROW_WRITE_ADMITTED = "ADMITTED"
ROW_WRITE_RETRYABLE = "RETRYABLE"
ROW_WRITE_UNAVAILABLE = "UNAVAILABLE"


@dataclass(frozen=True)
class RowWriteAdmission:
    state: str
    reason: str = ""
    row_index: int | None = None
    lease: tuple[int, int] | None = None

    def __post_init__(self) -> None:
        if self.state not in {
            ROW_WRITE_ADMITTED,
            ROW_WRITE_RETRYABLE,
            ROW_WRITE_UNAVAILABLE,
        }:
            raise ValueError(f"unsupported row write admission state: {self.state}")
        if self.lease is not None and (
            self.row_index is None or self.lease[1] != self.row_index
        ):
            raise ValueError("row write lease must identify row_index")

    @property
    def admitted(self) -> bool:
        return self.state == ROW_WRITE_ADMITTED

STREAM_PATHS: dict[str, Path] = {
    "events": Path("events.jsonl"),
    "measurements": Path("measurements.jsonl"),
    "method_evaluations": Path("method_evaluations.jsonl"),
    "camera.left": Path("camera_left.jsonl"),
    "camera.right": Path("camera_right.jsonl"),
    "px4.left": Path("px4_left.jsonl"),
    "px4.right": Path("px4_right.jsonl"),
    "px4.drone": Path("px4_drone.jsonl"),
    "gnss.base": Path("gnss_base.jsonl"),
    "geometry.comparisons": Path("geometry_comparisons.jsonl"),
    "capture.events": Path("capture_events.jsonl"),
    "alignment": Path("alignment.jsonl"),
    "artifacts": Path("artifacts.jsonl"),
    "application.log": Path("application_log.jsonl"),
    "native.frames": Path("native_frames.jsonl"),
}

PRIORITY_STREAMS = frozenset(
    {
        "events",
        "measurements",
        "method_evaluations",
        "capture.events",
        "alignment",
        "artifacts",
        "native.frames",
    }
)
ESSENTIAL_RETRY_STREAMS = frozenset(
    {"events", "measurements", "method_evaluations"}
)
RETRY_RESERVE_MAX_BYTES = 8 * 1024 * 1024
_FSYNC = os.fsync

_SESSION_MEMBERSHIP_FIELDS = frozenset(
    {"session_id", "parent_session_id", "measurement_package_id"}
)
_SAFE_TOKEN = re.compile(r"[^A-Za-z0-9._-]+")
_KOREA_TIMEZONE = timezone(timedelta(hours=9), name="KST")


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if not (float("-inf") < value < float("inf")):
            raise ValueError("measurement data contains a non-finite number")
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {
            str(key): _json_safe(item)
            for key, item in value.items()
            if str(key) not in _SESSION_MEMBERSHIP_FIELDS
        }
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    to_mapping = getattr(value, "to_mapping", None)
    if callable(to_mapping):
        return _json_safe(to_mapping())
    raise TypeError(f"unsupported measurement data value: {type(value).__name__}")


def creation_timestamp_text(utc_ns: int | None = None) -> str:
    """Return a filename-safe local timestamp with an explicit UTC offset."""

    value = time.time_ns() if utc_ns is None else int(utc_ns)
    local = datetime.fromtimestamp(value / 1_000_000_000, tz=_KOREA_TIMEZONE)
    return f"{local:%Y%m%d_%H%M%S}_{value % 1_000_000_000:09d}{local:%z}"


def _safe_token(value: object, fallback: str) -> str:
    text = _SAFE_TOKEN.sub("_", str(value or "")).strip("._")
    return text or fallback


def allocate_timestamped_path(
    root: Path | str,
    category: str,
    *,
    label: str,
    suffix: str,
    utc_ns: int | None = None,
) -> Path:
    """Allocate a collision-free path without creating or replacing a file."""

    base = Path(root).expanduser().resolve(strict=False)
    category_token = _safe_token(category, "artifacts")
    label_token = _safe_token(label, "artifact")
    suffix_text = str(suffix or "").strip().lower()
    if not re.fullmatch(r"\.[a-z0-9]{1,12}", suffix_text):
        raise ValueError("artifact suffix must be a short lowercase extension")
    directory = base / category_token
    directory.mkdir(parents=True, exist_ok=True)
    stamp = creation_timestamp_text(utc_ns)
    stem = f"{stamp}_{label_token}"
    candidate = directory / f"{stem}{suffix_text}"
    sequence = 1
    while candidate.exists() or candidate.is_symlink():
        candidate = directory / f"{stem}-{sequence:02d}{suffix_text}"
        sequence += 1
    return candidate


@dataclass
class _StreamState:
    state: str = "IDLE"
    last_write_utc_ns: int | None = None
    last_error: str = ""
    written_records: int = 0
    dropped_records: int = 0


@dataclass(frozen=True)
class _QueuedRecord:
    stream: str
    payload: bytes
    enqueued_monotonic_ns: int
    sequence: int


@dataclass(frozen=True)
class MeasurementDataCheckpoint:
    """Durable read boundary captured without blocking data producers."""

    root: Path
    captured_utc_ns: int
    settled: bool
    pending_before_cutoff: int
    stream_byte_limits: Mapping[str, int]
    storage_snapshot: Mapping[str, Any]


class MeasurementDataStore:
    """One bounded, non-blocking writer for fixed Workstation data streams."""

    def __init__(
        self,
        root: Path | str,
        *,
        priority_capacity: int = 256,
        bulk_capacity: int = 4096,
        fsync_interval_s: float = 1.0,
        retry_reserve_capacity: int = 256,
        retry_reserve_max_bytes: int = RETRY_RESERVE_MAX_BYTES,
    ) -> None:
        self.root = Path(root).expanduser().resolve(strict=False)
        self.priority_capacity = max(1, int(priority_capacity))
        self.bulk_capacity = max(1, int(bulk_capacity))
        self.fsync_interval_s = max(0.05, float(fsync_interval_s))
        self.retry_reserve_capacity = max(1, int(retry_reserve_capacity))
        self.retry_reserve_max_bytes = max(1024, int(retry_reserve_max_bytes))
        self._priority: queue.Queue[_QueuedRecord] = queue.Queue(
            maxsize=self.priority_capacity
        )
        self._bulk: queue.Queue[_QueuedRecord] = queue.Queue(
            maxsize=self.bulk_capacity
        )
        self._states = {name: _StreamState() for name in STREAM_PATHS}
        self._handles: dict[str, Any] = {}
        self._state_lock = threading.RLock()
        self._lifecycle_condition = threading.Condition(self._state_lock)
        self._io_lock = threading.RLock()
        self._lifecycle_state = STORE_LIFECYCLE_OPEN
        self._lifecycle_rejected_records = 0
        self._active_enqueues = 0
        self._active_operations = 0
        self._next_sequence = 0
        self._pending_sequences: set[int] = set()
        self._retry_reserve: deque[_QueuedRecord] = deque()
        self._retry_reserve_bytes = 0
        self._retry_backoff_s = {name: 1.0 for name in STREAM_PATHS}
        self._next_retry_monotonic_s: dict[str, float | None] = {
            name: None for name in STREAM_PATHS
        }
        self._stop_requested = threading.Event()
        self._flush_requested = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="workstation-measurement-data-writer",
            daemon=True,
        )
        self._thread.start()

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def path_for(self, stream: str) -> Path:
        try:
            relative = STREAM_PATHS[str(stream)]
        except KeyError as exc:
            raise ValueError(f"unknown measurement data stream: {stream}") from exc
        return self.root / relative

    def _reject_enqueue_for_lifecycle_unlocked(self, stream: str) -> bool:
        state = self._states[stream]
        state.dropped_records += 1
        state.last_error = (
            f"enqueue rejected: store lifecycle is {self._lifecycle_state}"
        )
        self._lifecycle_rejected_records += 1
        return False

    def enqueue(
        self,
        stream: str,
        value: Mapping[str, Any],
        *,
        priority: bool | None = None,
    ) -> bool:
        name = str(stream)
        if name not in STREAM_PATHS:
            raise ValueError(f"unknown measurement data stream: {name}")
        with self._state_lock:
            if self._lifecycle_state != STORE_LIFECYCLE_OPEN:
                return self._reject_enqueue_for_lifecycle_unlocked(name)
        now_utc_ns = time.time_ns()
        now_monotonic_ns = time.monotonic_ns()
        try:
            record = {
                str(key): _json_safe(item)
                for key, item in value.items()
                if str(key) not in _SESSION_MEMBERSHIP_FIELDS
            }
            record.setdefault("schema", DATA_RECORD_SCHEMA)
            record.setdefault("recorded_utc_ns", now_utc_ns)
            record.setdefault("recorded_monotonic_ns", now_monotonic_ns)
            encoded = (
                json.dumps(
                    record,
                    ensure_ascii=False,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
                + b"\n"
            )
        except (TypeError, ValueError) as exc:
            # A malformed producer record is local to that record. Pausing the
            # whole type stream would let one optional field suppress every
            # later valid Measurement result until an operator retry.
            with self._state_lock:
                if self._lifecycle_state != STORE_LIFECYCLE_OPEN:
                    return self._reject_enqueue_for_lifecycle_unlocked(name)
                state = self._states[name]
                state.dropped_records += 1
                state.last_error = f"record rejected: {exc}"
            return False
        with self._lifecycle_condition:
            if self._lifecycle_state != STORE_LIFECYCLE_OPEN:
                return self._reject_enqueue_for_lifecycle_unlocked(name)
            self._active_enqueues += 1
            self._active_operations += 1
            self._next_sequence += 1
            sequence = self._next_sequence
            self._pending_sequences.add(sequence)
        queued = _QueuedRecord(name, encoded, now_monotonic_ns, sequence)
        try:
            with self._state_lock:
                retry_stream_pending = any(
                    item.stream == name for item in self._retry_reserve
                )
                if self._states[name].state == "FAILED" or retry_stream_pending:
                    accepted = self._reserve_retry_unlocked(queued)
                    if not accepted:
                        self._settle_sequence_unlocked(queued.sequence)
                    return accepted
            use_priority = (
                name in PRIORITY_STREAMS if priority is None else bool(priority)
            )
            target = self._priority if use_priority else self._bulk
            try:
                target.put_nowait(queued)
            except queue.Full:
                if use_priority:
                    with self._state_lock:
                        self._states[name].dropped_records += 1
                        self._settle_sequence_unlocked(queued.sequence)
                    return False
                try:
                    evicted = target.get_nowait()
                except queue.Empty:
                    with self._state_lock:
                        self._states[name].dropped_records += 1
                        self._settle_sequence_unlocked(queued.sequence)
                    return False
                with self._state_lock:
                    self._states[evicted.stream].dropped_records += 1
                    self._settle_sequence_unlocked(evicted.sequence)
                try:
                    target.put_nowait(queued)
                except queue.Full:
                    # Another producer can claim the slot between the eviction
                    # and this non-blocking admission. Treat that race as a
                    # factual drop instead of leaking queue.Full through a
                    # camera/telemetry callback.
                    with self._state_lock:
                        self._states[name].dropped_records += 1
                        self._settle_sequence_unlocked(queued.sequence)
                    return False
            with self._state_lock:
                if self._states[name].state == "IDLE":
                    self._states[name].state = "PENDING"
            return True
        finally:
            with self._lifecycle_condition:
                self._active_enqueues -= 1
                self._active_operations -= 1
                self._lifecycle_condition.notify_all()

    def retry_failed(self) -> None:
        with self._lifecycle_condition:
            if self._lifecycle_state != STORE_LIFECYCLE_OPEN:
                return
            self._active_operations += 1
        try:
            with self._state_lock:
                failed = [
                    name
                    for name, state in self._states.items()
                    if state.state == "FAILED"
                ]
                for name in failed:
                    self._states[name].state = "IDLE"
                    self._states[name].last_error = ""
                    self._close_handle(name)
                for name in failed:
                    self._next_retry_monotonic_s[name] = None
                    self._retry_backoff_s[name] = 1.0
            self._drain_retry_reserve()
        finally:
            with self._lifecycle_condition:
                self._active_operations -= 1
                self._lifecycle_condition.notify_all()

    def request_flush(self) -> None:
        self._flush_requested.set()

    def export_checkpoint(self, *, timeout_s: float = 5.0) -> MeasurementDataCheckpoint:
        """Flush and return complete-file byte limits for a background export.

        The cutoff covers records admitted before this call. Producers remain
        non-blocking; when retryable storage cannot settle within ``timeout_s``,
        the returned boundary is explicitly partial instead of delaying a
        Measurement command.
        """

        timeout = max(0.0, float(timeout_s))
        deadline = time.monotonic() + timeout
        with self._lifecycle_condition:
            if self._lifecycle_state != STORE_LIFECYCLE_OPEN:
                raise RuntimeError(
                    f"cannot checkpoint store lifecycle {self._lifecycle_state}"
                )
            self._active_operations += 1
            cutoff = self._next_sequence
        try:
            with self._lifecycle_condition:
                while True:
                    pending = sum(
                        1 for sequence in self._pending_sequences if sequence <= cutoff
                    )
                    if pending == 0:
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    self._lifecycle_condition.wait(timeout=min(0.05, remaining))
                settled = pending == 0
            with self._io_lock:
                self._flush_all()
                stream_byte_limits: dict[str, int] = {}
                for stream in STREAM_PATHS:
                    path = self.path_for(stream)
                    stream_byte_limits[stream] = (
                        path.stat().st_size
                        if path.is_file() and not path.is_symlink()
                        else 0
                    )
            return MeasurementDataCheckpoint(
                root=self.root,
                captured_utc_ns=time.time_ns(),
                settled=settled,
                pending_before_cutoff=pending,
                stream_byte_limits=stream_byte_limits,
                storage_snapshot=self.snapshot(),
            )
        finally:
            with self._lifecycle_condition:
                self._active_operations -= 1
                self._lifecycle_condition.notify_all()

    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            streams = {
                name: {
                    "path": str(self.path_for(name)),
                    "state": state.state,
                    "last_write_utc_ns": state.last_write_utc_ns,
                    "last_error": state.last_error,
                    "written_records": state.written_records,
                    "dropped_records": state.dropped_records,
                    "priority": name in PRIORITY_STREAMS,
                }
                for name, state in self._states.items()
            }
            retry_pending = len(self._retry_reserve)
            retry_bytes = self._retry_reserve_bytes
            retry_deadlines = {
                name: value
                for name, value in self._next_retry_monotonic_s.items()
                if value is not None
            }
            next_retry = min(retry_deadlines.values(), default=None)
            lifecycle_state = self._lifecycle_state
            lifecycle_rejected_records = self._lifecycle_rejected_records
            active_enqueues = self._active_enqueues
            active_operations = self._active_operations
            checkpoint_pending_records = len(self._pending_sequences)
        failed = sum(1 for value in streams.values() if value["state"] == "FAILED")
        dropped = sum(int(value["dropped_records"]) for value in streams.values())
        writer_alive = self.alive
        overall = (
            "UNAVAILABLE"
            if not writer_alive or failed == len(streams)
            else "DEGRADED"
            if failed or dropped
            else "OK"
        )
        return {
            "schema": DATA_STORE_SCHEMA,
            "state": overall,
            "lifecycle_state": lifecycle_state,
            "lifecycle_rejected_records": lifecycle_rejected_records,
            "active_enqueues": active_enqueues,
            "active_operations": active_operations,
            "checkpoint_pending_records": checkpoint_pending_records,
            "root": str(self.root),
            "priority_pending": self._priority.qsize(),
            "priority_capacity": self.priority_capacity,
            "bulk_pending": self._bulk.qsize(),
            "bulk_capacity": self.bulk_capacity,
            "dropped_records": dropped,
            "writer_alive": writer_alive,
            "essential_retry_pending": retry_pending,
            "essential_retry_bytes": retry_bytes,
            "next_retry_monotonic_s": next_retry,
            "next_retry_by_stream": retry_deadlines,
            "streams": streams,
        }

    def close(self, *, timeout_s: float = 2.0) -> bool:
        timeout = max(0.0, float(timeout_s))
        deadline = time.monotonic() + timeout
        with self._lifecycle_condition:
            if self._lifecycle_state == STORE_LIFECYCLE_CLOSED:
                return True
            if self._lifecycle_state == STORE_LIFECYCLE_OPEN:
                self._lifecycle_state = STORE_LIFECYCLE_CLOSING
            while self._active_operations:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._lifecycle_condition.wait(timeout=remaining)
            self._stop_requested.set()
            self._flush_requested.set()
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if self._thread.is_alive():
            return False
        return True

    def _next_record(self) -> _QueuedRecord | None:
        try:
            return self._priority.get_nowait()
        except queue.Empty:
            pass
        try:
            return self._bulk.get(timeout=0.05)
        except queue.Empty:
            return None

    def _run(self) -> None:
        next_fsync = time.monotonic() + self.fsync_interval_s
        try:
            while True:
                if (
                    self._stop_requested.is_set()
                    and self._priority.empty()
                    and self._bulk.empty()
                ):
                    break
                self._maybe_retry_failed()
                self._drain_retry_reserve()
                item = self._next_record()
                if item is not None:
                    self._write(item)
                now_s = time.monotonic()
                if self._flush_requested.is_set() or now_s >= next_fsync:
                    self._flush_all()
                    self._flush_requested.clear()
                    next_fsync = now_s + self.fsync_interval_s
        finally:
            self._flush_all()
            with self._state_lock:
                while self._retry_reserve:
                    item = self._retry_reserve.popleft()
                    self._retry_reserve_bytes -= len(item.payload)
                    self._states[item.stream].dropped_records += 1
                    self._settle_sequence_unlocked(item.sequence)
                for name in tuple(self._handles):
                    self._close_handle(name)
                self._lifecycle_state = STORE_LIFECYCLE_CLOSED

    def _write(self, item: _QueuedRecord) -> None:
        with self._state_lock:
            if self._states[item.stream].state == "FAILED":
                accepted = self._reserve_retry_unlocked(item)
                if not accepted:
                    self._settle_sequence_unlocked(item.sequence)
                return
        try:
            with self._io_lock:
                handle = self._handle(item.stream)
                written = handle.write(item.payload)
            if written != len(item.payload):
                raise OSError(
                    f"short append: expected={len(item.payload)}, actual={written}"
                )
        except OSError as exc:
            self._mark_failed(item.stream, str(exc))
            with self._state_lock:
                accepted = self._reserve_retry_unlocked(item)
                if not accepted:
                    self._settle_sequence_unlocked(item.sequence)
            return
        with self._state_lock:
            state = self._states[item.stream]
            state.state = "ACTIVE"
            state.last_error = ""
            state.last_write_utc_ns = time.time_ns()
            state.written_records += 1
            self._retry_backoff_s[item.stream] = 1.0
            self._next_retry_monotonic_s[item.stream] = None
            self._settle_sequence_unlocked(item.sequence)

    def _handle(self, stream: str):
        with self._state_lock:
            existing = self._handles.get(stream)
            if existing is not None:
                return existing
            path = self.path_for(stream)
            path.parent.mkdir(parents=True, exist_ok=True)
            existed = path.exists()
            if path.is_symlink():
                raise OSError(f"measurement data stream must not be a symlink: {path}")
            if existed and not path.is_file():
                raise OSError(f"measurement data stream is not a regular file: {path}")
            handle = path.open("ab", buffering=0)
            if existed and path.stat().st_size:
                with path.open("rb") as reader:
                    reader.seek(-1, os.SEEK_END)
                    if reader.read(1) != b"\n":
                        handle.write(b"\n")
            self._handles[stream] = handle
            return handle

    def _mark_failed(self, stream: str, error: str) -> None:
        with self._state_lock:
            state = self._states[stream]
            state.state = "FAILED"
            state.last_error = str(error)
            self._close_handle(stream)
            if self._next_retry_monotonic_s[stream] is None:
                self._next_retry_monotonic_s[stream] = (
                    time.monotonic() + self._retry_backoff_s[stream]
                )

    def _reserve_retry_unlocked(
        self,
        item: _QueuedRecord,
    ) -> bool:
        if item.stream not in ESSENTIAL_RETRY_STREAMS:
            self._states[item.stream].dropped_records += 1
            return False
        item_bytes = len(item.payload)
        if item_bytes > self.retry_reserve_max_bytes:
            self._states[item.stream].dropped_records += 1
            return False
        # Items can arrive here by two paths: a producer can observe FAILED
        # and reserve a new record directly, while older records already in
        # the writer queue reach the reserve later. Keep enqueue-time order so
        # the direct path cannot overtake those older records.
        insert_at = len(self._retry_reserve)
        for index, pending in enumerate(self._retry_reserve):
            if item.enqueued_monotonic_ns < pending.enqueued_monotonic_ns:
                insert_at = index
                break
        self._retry_reserve.insert(insert_at, item)
        self._retry_reserve_bytes += item_bytes
        accepted = True
        while self._retry_reserve and (
            len(self._retry_reserve) > self.retry_reserve_capacity
            or self._retry_reserve_bytes > self.retry_reserve_max_bytes
        ):
            evicted = self._retry_reserve.pop()
            self._retry_reserve_bytes -= len(evicted.payload)
            self._states[evicted.stream].dropped_records += 1
            self._settle_sequence_unlocked(evicted.sequence)
            if evicted is item:
                accepted = False
        return accepted

    def _settle_sequence_unlocked(self, sequence: int) -> None:
        self._pending_sequences.discard(int(sequence))
        self._lifecycle_condition.notify_all()

    def _maybe_retry_failed(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            due_streams = {
                name
                for name, due in self._next_retry_monotonic_s.items()
                if due is not None and now >= due
            }
            if not due_streams:
                return
            for name, state in self._states.items():
                if name in due_streams and state.state == "FAILED":
                    state.state = "IDLE"
                    state.last_error = ""
                    self._close_handle(name)
                if name in due_streams:
                    self._next_retry_monotonic_s[name] = None
                    self._retry_backoff_s[name] = min(
                        30.0, self._retry_backoff_s[name] * 2.0
                    )

    def _drain_retry_reserve(self) -> None:
        while True:
            with self._state_lock:
                if not self._retry_reserve:
                    return
                candidate_index = next(
                    (
                        index
                        for index, pending in enumerate(self._retry_reserve)
                        if self._states[pending.stream].state != "FAILED"
                    ),
                    None,
                )
                if candidate_index is None:
                    return
                item = self._retry_reserve[candidate_index]
                try:
                    self._priority.put_nowait(item)
                except queue.Full:
                    return
                if (
                    candidate_index < len(self._retry_reserve)
                    and self._retry_reserve[candidate_index] is item
                ):
                    del self._retry_reserve[candidate_index]
                    self._retry_reserve_bytes -= len(item.payload)

    def _flush_all(self) -> None:
        with self._io_lock:
            with self._state_lock:
                handles = tuple(self._handles.items())
            for stream, handle in handles:
                try:
                    handle.flush()
                    _FSYNC(handle.fileno())
                except OSError as exc:
                    self._mark_failed(stream, f"flush failed: {exc}")

    def _close_handle(self, stream: str) -> None:
        handle = self._handles.pop(stream, None)
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass


class LiveDataRows(Sequence[dict[str, Any]]):
    """Current-run in-memory rows with best-effort append persistence.

    Updates remain append-only on disk: the later snapshot carries the same
    row index and `_record_operation=replace`, so no historical bytes need to
    be rewritten before the next Measurement can start.
    """

    def __init__(
        self,
        store: MeasurementDataStore,
        stream: str,
        *,
        preview_limit: int = 256,
        priority: bool | None = None,
    ) -> None:
        self.store = store
        self.stream = str(stream)
        self.preview_limit = max(1, int(preview_limit))
        self.priority = priority
        self._rows: list[dict[str, Any]] = []
        self._lock = threading.RLock()

    @property
    def preview_rows(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(row) for row in self._rows[-self.preview_limit :]]

    @property
    def write_error(self) -> str | None:
        state = self.store.snapshot()["streams"].get(self.stream, {})
        return str(state.get("last_error") or "") or None

    def append(self, row: Mapping[str, Any]) -> bool:
        with self._lock:
            stored = dict(_json_safe(dict(row)))
            index = len(self._rows)
            self._rows.append(stored)
        return self.store.enqueue(
            self.stream,
            {
                **stored,
                "_record_operation": "append",
                "_record_index": index,
            },
            priority=self.priority,
        )

    def replace(self, index: int, row: Mapping[str, Any]) -> bool:
        with self._lock:
            resolved = index if index >= 0 else len(self._rows) + index
            if resolved < 0 or resolved >= len(self._rows):
                raise IndexError(index)
            stored = dict(_json_safe(dict(row)))
            self._rows[resolved] = stored
        return self.store.enqueue(
            self.stream,
            {
                **stored,
                "_record_operation": "replace",
                "_record_index": resolved,
            },
            priority=self.priority,
        )

    def close(self) -> bool:
        self.store.request_flush()
        return True

    def flush(self) -> bool:
        self.store.request_flush()
        return True

    def clear(self) -> None:
        with self._lock:
            self._rows.clear()

    def discard_spool(self) -> bool:
        return self.close()

    def __len__(self) -> int:
        with self._lock:
            return len(self._rows)

    def __iter__(self) -> Iterator[dict[str, Any]]:
        with self._lock:
            rows = [dict(row) for row in self._rows]
        yield from rows

    def __getitem__(self, index):
        with self._lock:
            if isinstance(index, slice):
                return [dict(row) for row in self._rows[index]]
            return dict(self._rows[index])

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Sequence) and list(self) == list(other)


__all__ = [
    "DATA_RECORD_SCHEMA",
    "DATA_STORE_SCHEMA",
    "ESSENTIAL_RETRY_STREAMS",
    "MeasurementDataCheckpoint",
    "MeasurementDataStore",
    "LiveDataRows",
    "PRIORITY_STREAMS",
    "STREAM_PATHS",
    "allocate_timestamped_path",
    "creation_timestamp_text",
]
