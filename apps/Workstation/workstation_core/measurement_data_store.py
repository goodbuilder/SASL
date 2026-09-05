"""Availability-first append storage for Workstation measurement data.

The store deliberately owns no Measurement or Session lifecycle.  Producers
 enqueue independently timestamped records and continue even when a stream is
 unwritable or overloaded.  One background thread is the only writer for the
 type-owned files below.
"""

from __future__ import annotations

import csv
import json
import os
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Mapping
from collections.abc import Iterator, Sequence


DATA_STORE_SCHEMA = "workstation-measurement-data-store-v1"
DATA_RECORD_SCHEMA = "workstation-measurement-data-record-v1"
CSV_RECORD_SCHEMA = "workstation-measurement-csv-v1"
CSV_FIELDNAMES = (
    "csv_schema",
    "recorded_utc_ns",
    "recorded_monotonic_ns",
    "runtime_generation",
    "measurement_id",
    "event_type",
    "sample_type",
    "status",
    "record_json",
)
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
    "gnss.baseline.center": Path("gnss_baseline_center.jsonl"),
    "geometry.comparisons": Path("geometry_comparisons.jsonl"),
    "capture.events": Path("capture_events.jsonl"),
    "alignment": Path("alignment.jsonl"),
    "artifacts": Path("artifacts.jsonl"),
    "application.log": Path("application_log.jsonl"),
    "native.frames": Path("native_frames.jsonl"),
}
CSV_STREAM_PATHS: dict[str, Path] = {
    stream: relative.with_suffix(".csv")
    for stream, relative in STREAM_PATHS.items()
}


def timestamped_stream_path(stream: str, stamp: str) -> Path:
    """Return one timestamped filename for a logical data stream."""

    try:
        relative = STREAM_PATHS[str(stream)]
    except KeyError as exc:
        raise ValueError(f"unknown measurement data stream: {stream}") from exc
    token = str(stamp or "").strip()
    if not token or any(character in token for character in ("/", "\\", "\x00")):
        raise ValueError("stream filename stamp must be one safe path token")
    return relative.with_name(f"{relative.stem}_{token}{relative.suffix}")


def timestamped_csv_path(stream: str, stamp: str) -> Path:
    """Return the same-stem CSV mirror path for one logical stream."""

    return timestamped_stream_path(stream, stamp).with_suffix(".csv")


def _unique_stream_file_stamp(root: Path, stamp: str) -> str:
    """Avoid reusing a timestamped file set if a clock value collides."""

    base = str(stamp)
    candidate = base
    sequence = 1
    while any(
        (root / timestamped_stream_path(stream, candidate)).exists()
        or (root / timestamped_stream_path(stream, candidate)).is_symlink()
        or (root / timestamped_csv_path(stream, candidate)).exists()
        or (root / timestamped_csv_path(stream, candidate)).is_symlink()
        for stream in STREAM_PATHS
    ):
        candidate = f"{base}-{sequence:02d}"
        sequence += 1
    return candidate

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


@dataclass
class _CsvState:
    state: str = "IDLE"
    last_write_utc_ns: int | None = None
    last_error: str = ""
    written_records: int = 0
    dropped_records: int = 0


class _CsvHandle:
    """One validated text handle and writer owned by the storage thread."""

    def __init__(self, handle: Any, writer: csv.DictWriter) -> None:
        self.handle = handle
        self.writer = writer

    def write(self, row: Mapping[str, Any]) -> None:
        self.writer.writerow(dict(row))

    def flush(self) -> None:
        self.handle.flush()
        _FSYNC(self.handle.fileno())

    def close(self) -> None:
        self.handle.close()


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
    stream_paths: Mapping[str, tuple[Path, ...]] = field(default_factory=dict)
    stream_byte_limits_by_path: Mapping[str, int] = field(default_factory=dict)


class MeasurementDataStore:
    """One bounded, non-blocking writer for Workstation data streams."""

    def __init__(
        self,
        root: Path | str,
        *,
        priority_capacity: int = 256,
        bulk_capacity: int = 4096,
        fsync_interval_s: float = 1.0,
        retry_reserve_capacity: int = 256,
        retry_reserve_max_bytes: int = RETRY_RESERVE_MAX_BYTES,
        timestamped_filenames: bool = False,
        file_stamp: str | None = None,
    ) -> None:
        self.root = Path(root).expanduser().resolve(strict=False)
        self.timestamped_filenames = bool(timestamped_filenames)
        self.file_stamp = (
            _unique_stream_file_stamp(
                self.root,
                str(file_stamp or creation_timestamp_text()),
            )
            if self.timestamped_filenames
            else ""
        )
        self._stream_paths = {
            stream: (
                self.root / timestamped_stream_path(stream, self.file_stamp)
                if self.timestamped_filenames
                else self.root / relative
            )
            for stream, relative in STREAM_PATHS.items()
        }
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
        self._csv_states = {name: _CsvState() for name in STREAM_PATHS}
        self._handles: dict[str, Any] = {}
        self._csv_handles: dict[str, _CsvHandle] = {}
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
        self._csv_retry_reserve: deque[_QueuedRecord] = deque()
        self._csv_retry_reserve_bytes = 0
        self._retry_backoff_s = {name: 1.0 for name in STREAM_PATHS}
        self._csv_retry_backoff_s = {name: 1.0 for name in STREAM_PATHS}
        self._next_retry_monotonic_s: dict[str, float | None] = {
            name: None for name in STREAM_PATHS
        }
        self._csv_next_retry_monotonic_s: dict[str, float | None] = {
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
            return self._stream_paths[str(stream)]
        except KeyError as exc:
            raise ValueError(f"unknown measurement data stream: {stream}") from exc

    def csv_path_for(self, stream: str) -> Path:
        """Return the same-stem CSV mirror path for one logical stream."""

        name = str(stream)
        if name not in CSV_STREAM_PATHS:
            raise ValueError(f"unknown measurement data stream: {name}")
        return self.path_for(name).with_suffix(".csv")

    def stream_paths_for_export(self, stream: str) -> tuple[Path, ...]:
        """Return current and legacy files that may contain a stream."""

        try:
            relative = STREAM_PATHS[str(stream)]
        except KeyError as exc:
            raise ValueError(f"unknown measurement data stream: {stream}") from exc
        candidates = [self.path_for(stream), self.root / relative]
        if self.timestamped_filenames:
            candidates.extend(
                sorted(
                    self.root.glob(f"{relative.stem}_*{relative.suffix}"),
                    key=lambda path: path.name,
                )
            )
        unique: list[Path] = []
        for candidate in candidates:
            if candidate not in unique:
                unique.append(candidate)
        return tuple(unique)

    def csv_paths_for_export(self, stream: str) -> tuple[Path, ...]:
        """Return current, legacy, and prior same-stem CSV mirror files."""

        name = str(stream)
        try:
            relative = CSV_STREAM_PATHS[name]
        except KeyError as exc:
            raise ValueError(f"unknown measurement data stream: {name}") from exc
        candidates = [self.csv_path_for(name), self.root / relative]
        if self.timestamped_filenames:
            candidates.extend(
                sorted(
                    self.root.glob(f"{relative.stem}_*{relative.suffix}"),
                    key=lambda path: path.name,
                )
            )
        unique: list[Path] = []
        for candidate in candidates:
            if candidate not in unique:
                unique.append(candidate)
        return tuple(unique)

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
                csv_failed = [
                    name
                    for name, state in self._csv_states.items()
                    if state.state == "FAILED"
                ]
                for name in failed:
                    self._states[name].state = "IDLE"
                    self._states[name].last_error = ""
                    self._close_handle(name)
                for name in csv_failed:
                    self._csv_states[name].state = "IDLE"
                    self._csv_states[name].last_error = ""
                    self._close_csv_handle(name)
                for name in failed:
                    self._next_retry_monotonic_s[name] = None
                    self._retry_backoff_s[name] = 1.0
                for name in csv_failed:
                    self._csv_next_retry_monotonic_s[name] = None
                    self._csv_retry_backoff_s[name] = 1.0
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
                stream_paths: dict[str, tuple[Path, ...]] = {}
                stream_byte_limits_by_path: dict[str, int] = {}
                for stream in STREAM_PATHS:
                    paths = self.stream_paths_for_export(stream)
                    stream_paths[stream] = paths
                    current = self.path_for(stream)
                    for path in paths:
                        size = (
                            path.stat().st_size
                            if path.is_file() and not path.is_symlink()
                            else 0
                        )
                        try:
                            relative = path.relative_to(self.root).as_posix()
                        except ValueError:
                            relative = path.as_posix()
                        stream_byte_limits_by_path[relative] = size
                    stream_byte_limits[stream] = stream_byte_limits_by_path.get(
                        current.relative_to(self.root).as_posix(),
                        0,
                    )
            return MeasurementDataCheckpoint(
                root=self.root,
                captured_utc_ns=time.time_ns(),
                settled=settled,
                pending_before_cutoff=pending,
                stream_byte_limits=stream_byte_limits,
                storage_snapshot=self.snapshot(),
                stream_paths=stream_paths,
                stream_byte_limits_by_path=stream_byte_limits_by_path,
            )
        finally:
            with self._lifecycle_condition:
                self._active_operations -= 1
                self._lifecycle_condition.notify_all()

    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            csv_retry_pending_by_stream = {name: 0 for name in STREAM_PATHS}
            for item in self._csv_retry_reserve:
                csv_retry_pending_by_stream[item.stream] += 1
            streams: dict[str, dict[str, Any]] = {}
            for name, state in self._states.items():
                csv_state = self._csv_states[name]
                streams[name] = {
                    "path": str(self.path_for(name)),
                    "paths": [
                        str(path) for path in self.stream_paths_for_export(name)
                    ],
                    "filename_mode": (
                        "timestamped" if self.timestamped_filenames else "fixed"
                    ),
                    "file_stamp": self.file_stamp,
                    "state": state.state,
                    "last_write_utc_ns": state.last_write_utc_ns,
                    "last_error": state.last_error,
                    "written_records": state.written_records,
                    "dropped_records": state.dropped_records,
                    "priority": name in PRIORITY_STREAMS,
                    "csv": {
                        "path": str(self.csv_path_for(name)),
                        "paths": [
                            str(path) for path in self.csv_paths_for_export(name)
                        ],
                        "state": csv_state.state,
                        "last_write_utc_ns": csv_state.last_write_utc_ns,
                        "last_error": csv_state.last_error,
                        "written_records": csv_state.written_records,
                        "dropped_records": csv_state.dropped_records,
                        "retry_pending": csv_retry_pending_by_stream[name],
                    },
                }
            retry_pending = len(self._retry_reserve)
            retry_bytes = self._retry_reserve_bytes
            retry_deadlines = {
                name: value
                for name, value in self._next_retry_monotonic_s.items()
                if value is not None
            }
            next_retry = min(retry_deadlines.values(), default=None)
            csv_retry_pending = len(self._csv_retry_reserve)
            csv_retry_bytes = self._csv_retry_reserve_bytes
            csv_retry_deadlines = {
                name: value
                for name, value in self._csv_next_retry_monotonic_s.items()
                if value is not None
            }
            csv_next_retry = min(csv_retry_deadlines.values(), default=None)
            lifecycle_state = self._lifecycle_state
            lifecycle_rejected_records = self._lifecycle_rejected_records
            active_enqueues = self._active_enqueues
            active_operations = self._active_operations
            checkpoint_pending_records = len(self._pending_sequences)
        failed = sum(1 for value in streams.values() if value["state"] == "FAILED")
        dropped = sum(int(value["dropped_records"]) for value in streams.values())
        csv_failed = sum(
            1 for value in streams.values() if value["csv"]["state"] == "FAILED"
        )
        csv_dropped = sum(
            int(value["csv"]["dropped_records"]) for value in streams.values()
        )
        writer_alive = self.alive
        overall = (
            "UNAVAILABLE"
            if not writer_alive or failed == len(streams)
            else "DEGRADED"
            if failed or dropped or csv_failed or csv_dropped
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
            "csv_retry_pending": csv_retry_pending,
            "csv_retry_bytes": csv_retry_bytes,
            "csv_dropped_records": csv_dropped,
            "csv_next_retry_monotonic_s": csv_next_retry,
            "csv_next_retry_by_stream": csv_retry_deadlines,
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
                self._drain_csv_retry_reserve()
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
                while self._csv_retry_reserve:
                    item = self._csv_retry_reserve.popleft()
                    self._csv_retry_reserve_bytes -= len(item.payload)
                    self._csv_states[item.stream].dropped_records += 1
                for name in tuple(self._handles):
                    self._close_handle(name)
                for name in tuple(self._csv_handles):
                    self._close_csv_handle(name)
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
        self._write_csv_or_reserve(item)

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

    @staticmethod
    def _csv_row_from_payload(payload: bytes) -> dict[str, Any]:
        try:
            record_json = payload.rstrip(b"\n").decode("utf-8")
            record = json.loads(record_json)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise OSError(f"canonical JSONL payload cannot be mirrored to CSV: {exc}") from exc
        if not isinstance(record, Mapping):
            raise OSError("canonical JSONL payload is not an object")

        def scalar(name: str) -> Any:
            value = record.get(name, "")
            if value is None or isinstance(value, (bool, int, float, str)):
                return "" if value is None else value
            try:
                return json.dumps(
                    value,
                    ensure_ascii=False,
                    allow_nan=False,
                    sort_keys=True,
                    separators=(",", ":"),
                )
            except (TypeError, ValueError) as exc:
                raise OSError(f"CSV envelope field {name} is not serializable: {exc}") from exc

        return {
            "csv_schema": CSV_RECORD_SCHEMA,
            "recorded_utc_ns": scalar("recorded_utc_ns"),
            "recorded_monotonic_ns": scalar("recorded_monotonic_ns"),
            "runtime_generation": scalar("runtime_generation"),
            "measurement_id": scalar("measurement_id"),
            "event_type": scalar("event_type"),
            "sample_type": scalar("sample_type"),
            "status": scalar("status"),
            "record_json": record_json,
        }

    def _write_csv_or_reserve(self, item: _QueuedRecord) -> None:
        with self._state_lock:
            csv_state = self._csv_states[item.stream]
            stream_pending = any(
                pending.stream == item.stream for pending in self._csv_retry_reserve
            )
            if csv_state.state == "FAILED" or stream_pending:
                self._reserve_csv_retry_unlocked(item)
                return
            csv_state.state = "PENDING"
        try:
            row = self._csv_row_from_payload(item.payload)
            with self._io_lock:
                handle = self._csv_handle(item.stream)
                handle.write(row)
        except (OSError, TypeError, ValueError, csv.Error) as exc:
            self._mark_csv_failed(item.stream, str(exc))
            with self._state_lock:
                self._reserve_csv_retry_unlocked(item)
            return
        with self._state_lock:
            state = self._csv_states[item.stream]
            state.state = "ACTIVE"
            state.last_error = ""
            state.last_write_utc_ns = time.time_ns()
            state.written_records += 1
            self._csv_retry_backoff_s[item.stream] = 1.0
            self._csv_next_retry_monotonic_s[item.stream] = None

    def _csv_handle(self, stream: str) -> _CsvHandle:
        with self._state_lock:
            existing = self._csv_handles.get(stream)
            if existing is not None:
                return existing
            path = self.csv_path_for(stream)
            path.parent.mkdir(parents=True, exist_ok=True)
            existed = path.exists()
            if path.is_symlink():
                raise OSError(f"CSV mirror must not be a symlink: {path}")
            if existed and not path.is_file():
                raise OSError(f"CSV mirror is not a regular file: {path}")
            size = path.stat().st_size if existed else 0
            if size:
                try:
                    with path.open("r", newline="", encoding="utf-8") as reader:
                        header = next(csv.reader(reader), None)
                except (OSError, UnicodeError, csv.Error) as exc:
                    raise OSError(f"CSV mirror header cannot be read: {path}: {exc}") from exc
                if header != list(CSV_FIELDNAMES):
                    raise OSError(f"CSV mirror header is invalid or foreign: {path}")
            handle: Any | None = None
            try:
                handle = path.open("a", newline="", encoding="utf-8")
                writer = csv.DictWriter(
                    handle,
                    fieldnames=CSV_FIELDNAMES,
                    lineterminator="\n",
                )
                if not size:
                    writer.writeheader()
                else:
                    with path.open("rb") as reader:
                        reader.seek(-1, os.SEEK_END)
                        if reader.read(1) != b"\n":
                            handle.write("\n")
                wrapped = _CsvHandle(handle, writer)
            except (OSError, UnicodeError, ValueError, csv.Error) as exc:
                if handle is not None:
                    try:
                        handle.close()
                    except (OSError, ValueError):
                        pass
                raise OSError(f"CSV mirror cannot be opened: {path}: {exc}") from exc
            self._csv_handles[stream] = wrapped
            return wrapped

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

    def _mark_csv_failed(self, stream: str, error: str) -> None:
        with self._state_lock:
            state = self._csv_states[stream]
            state.state = "FAILED"
            state.last_error = str(error)
            self._close_csv_handle(stream)
            if self._csv_next_retry_monotonic_s[stream] is None:
                self._csv_next_retry_monotonic_s[stream] = (
                    time.monotonic() + self._csv_retry_backoff_s[stream]
                )

    def _reserve_csv_retry_unlocked(
        self,
        item: _QueuedRecord,
    ) -> bool:
        item_bytes = len(item.payload)
        if item_bytes > self.retry_reserve_max_bytes:
            self._csv_states[item.stream].dropped_records += 1
            return False
        insert_at = len(self._csv_retry_reserve)
        for index, pending in enumerate(self._csv_retry_reserve):
            if item.enqueued_monotonic_ns < pending.enqueued_monotonic_ns:
                insert_at = index
                break
        self._csv_retry_reserve.insert(insert_at, item)
        self._csv_retry_reserve_bytes += item_bytes
        accepted = True
        while self._csv_retry_reserve and (
            len(self._csv_retry_reserve) > self.retry_reserve_capacity
            or self._csv_retry_reserve_bytes > self.retry_reserve_max_bytes
        ):
            evicted = self._csv_retry_reserve.pop()
            self._csv_retry_reserve_bytes -= len(evicted.payload)
            self._csv_states[evicted.stream].dropped_records += 1
            if evicted is item:
                accepted = False
        return accepted

    def _drain_csv_retry_reserve(self) -> None:
        with self._state_lock:
            candidate_index = next(
                (
                    index
                    for index, pending in enumerate(self._csv_retry_reserve)
                    if self._csv_states[pending.stream].state != "FAILED"
                ),
                None,
            )
            if candidate_index is None:
                return
            item = self._csv_retry_reserve[candidate_index]
            del self._csv_retry_reserve[candidate_index]
            self._csv_retry_reserve_bytes -= len(item.payload)
        self._write_csv_or_reserve(item)

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
            csv_due_streams = {
                name
                for name, due in self._csv_next_retry_monotonic_s.items()
                if due is not None and now >= due
            }
            for name in due_streams:
                state = self._states[name]
                if state.state == "FAILED":
                    state.state = "IDLE"
                    state.last_error = ""
                    self._close_handle(name)
                self._next_retry_monotonic_s[name] = None
                self._retry_backoff_s[name] = min(
                    30.0, self._retry_backoff_s[name] * 2.0
                )
            for name in csv_due_streams:
                state = self._csv_states[name]
                if state.state == "FAILED":
                    state.state = "IDLE"
                    state.last_error = ""
                    self._close_csv_handle(name)
                self._csv_next_retry_monotonic_s[name] = None
                self._csv_retry_backoff_s[name] = min(
                    30.0, self._csv_retry_backoff_s[name] * 2.0
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
                csv_handles = tuple(self._csv_handles.items())
            for stream, handle in handles:
                try:
                    handle.flush()
                    _FSYNC(handle.fileno())
                except OSError as exc:
                    self._mark_failed(stream, f"flush failed: {exc}")
            for stream, handle in csv_handles:
                try:
                    handle.flush()
                except OSError as exc:
                    self._mark_csv_failed(stream, f"flush failed: {exc}")

    def _close_handle(self, stream: str) -> None:
        handle = self._handles.pop(stream, None)
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass

    def _close_csv_handle(self, stream: str) -> None:
        handle = self._csv_handles.pop(stream, None)
        if handle is not None:
            try:
                handle.close()
            except (OSError, ValueError):
                pass


class LiveDataRows(Sequence[dict[str, Any]]):
    """Current-run in-memory rows with best-effort append persistence.

    Updates remain append-only on disk. A row uses ``replace`` only after an
    earlier enqueue for the same index was admitted; otherwise the latest
    snapshot retries that index as ``append`` so persistence never starts with
    an orphan replacement.
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
        self._admitted_indices: set[int] = set()
        self._owner_epoch = 0
        self._lock = threading.RLock()

    @property
    def preview_rows(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(row) for row in self._rows[-self.preview_limit :]]

    @property
    def write_error(self) -> str | None:
        state = self.store.snapshot()["streams"].get(self.stream, {})
        return str(state.get("last_error") or "") or None

    def _store_unavailable_reason(self) -> str:
        snapshot_reader = getattr(self.store, "snapshot", None)
        if not callable(snapshot_reader):
            return ""
        try:
            snapshot = snapshot_reader()
        except (OSError, RuntimeError, TypeError, ValueError):
            return "row store status is unavailable"
        if not isinstance(snapshot, Mapping):
            return "row store status is unavailable"
        lifecycle = str(snapshot.get("lifecycle_state") or "").upper()
        if lifecycle and lifecycle != STORE_LIFECYCLE_OPEN:
            return f"row store lifecycle is {lifecycle}"
        if snapshot.get("writer_alive") is False:
            return "row store writer is not alive"
        return ""

    def append_with_admission(
        self,
        row: Mapping[str, Any],
    ) -> RowWriteAdmission:
        with self._lock:
            stored = dict(_json_safe(dict(row)))
            index = len(self._rows)
            lease = (self._owner_epoch, index)
            admitted = self.store.enqueue(
                self.stream,
                {
                    **stored,
                    "_record_operation": "append",
                    "_record_index": index,
                },
                priority=self.priority,
            )
            unavailable_reason = "" if admitted else self._store_unavailable_reason()
            # A retryable row keeps this index so the next attempt is emitted
            # as the same logical append. An unavailable store never acquires
            # the row; its caller must reroute the record to the direct path.
            if admitted or not unavailable_reason:
                self._rows.append(stored)
            if admitted:
                self._admitted_indices.add(index)
                return RowWriteAdmission(
                    ROW_WRITE_ADMITTED,
                    row_index=index,
                    lease=lease,
                )
            return RowWriteAdmission(
                ROW_WRITE_UNAVAILABLE if unavailable_reason else ROW_WRITE_RETRYABLE,
                unavailable_reason or "row enqueue was not admitted",
                row_index=index,
                lease=lease,
            )

    def append(self, row: Mapping[str, Any]) -> bool:
        return self.append_with_admission(row).admitted

    def replace(self, index: int, row: Mapping[str, Any]) -> bool:
        with self._lock:
            resolved = index if index >= 0 else len(self._rows) + index
            if resolved < 0 or resolved >= len(self._rows):
                raise IndexError(index)
            stored = dict(_json_safe(dict(row)))
            self._rows[resolved] = stored
            operation = (
                "replace" if resolved in self._admitted_indices else "append"
            )
            admitted = self.store.enqueue(
                self.stream,
                {
                    **stored,
                    "_record_operation": operation,
                    "_record_index": resolved,
                },
                priority=self.priority,
            )
            if admitted:
                self._admitted_indices.add(resolved)
            return admitted

    def replace_with_admission(
        self,
        index: int,
        row: Mapping[str, Any],
        *,
        lease: tuple[int, int] | None = None,
    ) -> RowWriteAdmission:
        """Atomically replace one logical row and classify persistence admission."""

        with self._lock:
            resolved = index if index >= 0 else len(self._rows) + index
            current_lease = (self._owner_epoch, resolved)
            if resolved < 0 or resolved >= len(self._rows):
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    "row index is no longer available",
                    row_index=resolved,
                    lease=current_lease,
                )
            if lease is not None and lease != current_lease:
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    "row lease no longer owns this index",
                    row_index=resolved,
                    lease=current_lease,
                )
            stored = dict(_json_safe(dict(row)))
            operation = (
                "replace" if resolved in self._admitted_indices else "append"
            )
            admitted = self.store.enqueue(
                self.stream,
                {
                    **stored,
                    "_record_operation": operation,
                    "_record_index": resolved,
                },
                priority=self.priority,
            )
            unavailable_reason = "" if admitted else self._store_unavailable_reason()
            if admitted or not unavailable_reason:
                self._rows[resolved] = stored
            if admitted:
                self._admitted_indices.add(resolved)
                return RowWriteAdmission(
                    ROW_WRITE_ADMITTED,
                    row_index=resolved,
                    lease=current_lease,
                )
            return RowWriteAdmission(
                ROW_WRITE_UNAVAILABLE if unavailable_reason else ROW_WRITE_RETRYABLE,
                unavailable_reason or "row enqueue was not admitted",
                row_index=resolved,
                lease=current_lease,
            )

    def close(self) -> bool:
        self.store.request_flush()
        return True

    def flush(self) -> bool:
        self.store.request_flush()
        return True

    def clear(self) -> None:
        with self._lock:
            self._owner_epoch += 1
            self._rows.clear()
            self._admitted_indices.clear()

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
    "CSV_FIELDNAMES",
    "CSV_RECORD_SCHEMA",
    "CSV_STREAM_PATHS",
    "DATA_RECORD_SCHEMA",
    "DATA_STORE_SCHEMA",
    "ESSENTIAL_RETRY_STREAMS",
    "ROW_WRITE_ADMITTED",
    "ROW_WRITE_RETRYABLE",
    "ROW_WRITE_UNAVAILABLE",
    "RowWriteAdmission",
    "MeasurementDataCheckpoint",
    "MeasurementDataStore",
    "LiveDataRows",
    "PRIORITY_STREAMS",
    "STREAM_PATHS",
    "allocate_timestamped_path",
    "creation_timestamp_text",
    "timestamped_csv_path",
    "timestamped_stream_path",
]
