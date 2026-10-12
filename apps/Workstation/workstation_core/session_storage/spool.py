"""Session spool responsibility extracted without changing saved data contracts.

Position/attitude frames, axis aliases, units, timestamps, schema and hashes
retain the producer/recording contract. This module does not convert camera
origins, optical RDF, baseline-center FRD or world NED implicitly.
"""
from __future__ import annotations

import hashlib
import json
import os
import tempfile
import threading
from collections import deque
from collections.abc import Iterator, Mapping, Sequence
from pathlib import Path
from typing import Callable
from Workstation.workstation_core.measurement_data_store import ROW_WRITE_ADMITTED, ROW_WRITE_UNAVAILABLE, RowWriteAdmission
from . import values as _storage_values


class SpoolRows(Sequence[dict[str, object]]):
    """Disk-backed append-only rows with a bounded in-memory preview."""

    def __init__(
        self,
        *,
        preview_limit: int = 256,
        supports_updates: bool = False,
        on_write_error: Callable[[OSError], None] | None = None,
    ) -> None:
        self.preview_limit = max(1, int(preview_limit))
        self.supports_updates = supports_updates
        self._preview: deque[dict[str, object]] = deque(maxlen=self.preview_limit)
        self._path: Path | None = None
        self._handle = None
        self._count = 0
        self._lock = threading.RLock()
        self._recovery_warnings: list[str] = []
        self._recovery_artifacts: list[Path] = []
        self._on_write_error = on_write_error
        self._write_error: str | None = None
        self._volatile_appends: list[dict[str, object]] = []
        self._volatile_updates: dict[int, dict[str, object]] = {}
        self._volatile_deletes: list[int] = []
        self._owner_epoch = 0

    @property
    def spool_path(self) -> Path | None:
        return self._path

    @property
    def preview_count(self) -> int:
        return len(self._preview)

    @property
    def recovery_warnings(self) -> tuple[str, ...]:
        return tuple(self._recovery_warnings)

    @property
    def recovery_artifacts(self) -> tuple[Path, ...]:
        return tuple(self._recovery_artifacts)

    @property
    def write_error(self) -> str | None:
        return self._write_error

    def _record_write_error(self, exc: OSError) -> None:
        first = self._write_error is None
        self._write_error = str(exc)
        handle = self._handle
        self._handle = None
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass
        if first and self._on_write_error is not None:
            try:
                self._on_write_error(exc)
            except Exception:
                pass

    def bind(self, path: Path) -> None:
        with self._lock:
            self.close()
            self._owner_epoch += 1
            path.parent.mkdir(parents=True, exist_ok=True)
            path.unlink(missing_ok=True)
            self._path = path
            self._handle = path.open("a", encoding="utf-8", buffering=1)
            self._preview.clear()
            self._count = 0
            self._recovery_warnings.clear()
            self._recovery_artifacts.clear()
            self._write_error = None
            self._volatile_appends.clear()
            self._volatile_updates.clear()
            self._volatile_deletes.clear()

    def open_existing(self, path: Path) -> None:
        with self._lock:
            self.close()
            self._owner_epoch += 1
            self._path = path
            self._preview.clear()
            self._count = 0
            self._recovery_warnings.clear()
            self._recovery_artifacts.clear()
            self._write_error = None
            self._volatile_appends.clear()
            self._volatile_updates.clear()
            self._volatile_deletes.clear()
            if path.exists():
                self._repair_unterminated_tail(path)
                for row in self._iter_replayed_rows() if self.supports_updates else self._plain_rows():
                    self._preview.append(row)
                    self._count += 1
            path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = path.open("a", encoding="utf-8", buffering=1)

    @staticmethod
    def _atomic_replace_bytes(path: Path, payload: bytes) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".repair",
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "wb") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def _repair_unterminated_tail(self, path: Path) -> None:
        """Preserve a valid prefix and quarantine only a torn final write.

        A malformed newline-terminated row or malformed interior row is not a
        recoverable crash tail and remains a hard error when rows are parsed.
        """

        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            if not size:
                return
            handle.seek(-1, os.SEEK_END)
            if handle.read(1) == b"\n":
                return
            # Only the final unterminated record is retained. A recovered
            # multi-gigabyte spool with a complete last row allocates no copy.
            position = size
            chunks = []
            prefix_length = 0
            while position:
                count = min(position, 64 * 1024)
                position -= count
                handle.seek(position)
                block = handle.read(count)
                boundary = block.rfind(b"\n")
                if boundary >= 0:
                    prefix_length = position + boundary + 1
                    chunks.append(block[boundary + 1:])
                    break
                chunks.append(block)
            tail = b"".join(reversed(chunks))
        try:
            decoded = json.loads(tail)
        except (UnicodeDecodeError, json.JSONDecodeError):
            quarantine_root = path.parent / ".quarantine"
            quarantine_root.mkdir(parents=True, exist_ok=True)
            digest = hashlib.sha256(tail).hexdigest()
            quarantine = quarantine_root / f"{path.name}.{digest[:16]}.truncated-tail"
            if not quarantine.exists():
                self._atomic_replace_bytes(quarantine, tail)
            self._atomic_replace_prefix(path, prefix_length)
            self._recovery_artifacts.append(quarantine)
            self._recovery_warnings.append(
                f"quarantined truncated final JSONL record: {quarantine.name}"
            )
            return
        if not isinstance(decoded, dict):
            raise OSError(f"unterminated spool row is not an object: {path}")
        self._atomic_replace_prefix(path, size, suffix=b"\n")
        self._recovery_warnings.append("normalized valid unterminated final JSONL record")

    @staticmethod
    def _atomic_replace_prefix(path: Path, length: int, *, suffix: bytes = b"") -> None:
        descriptor, temporary_name = tempfile.mkstemp(
            dir=path.parent, prefix=f".{path.name}.", suffix=".repair",
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "wb") as target, path.open("rb") as source:
                remaining = length
                while remaining:
                    block = source.read(min(remaining, 64 * 1024))
                    if not block:
                        raise OSError("spool changed during atomic tail repair")
                    target.write(block)
                    remaining -= len(block)
                target.write(suffix)
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def append_with_admission(
        self,
        row: Mapping[str, object],
    ) -> RowWriteAdmission:
        """Append one logical row and return a lease-bound write classification."""

        stored = {key: _storage_values._serializable_value(value) for key, value in row.items()}
        with self._lock:
            index = self._count
            lease = (self._owner_epoch, index)
            if self._write_error is not None:
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    f"spool write failed: {self._write_error}",
                    row_index=index,
                    lease=lease,
                )
            if self._path is None or self._handle is None:
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    "spool is unbound or closed",
                    row_index=index,
                    lease=lease,
                )
            try:
                self._handle.write(
                    json.dumps(stored, ensure_ascii=False, sort_keys=True) + "\n"
                )
            except OSError as exc:
                self._record_write_error(exc)
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    f"spool write failed: {exc}",
                    row_index=index,
                    lease=lease,
                )
            self._preview.append(stored)
            self._count += 1
            return RowWriteAdmission(
                ROW_WRITE_ADMITTED,
                row_index=index,
                lease=lease,
            )

    def append(self, row: dict[str, object]) -> bool:
        stored = {key: _storage_values._serializable_value(value) for key, value in row.items()}
        persisted = True
        with self._lock:
            if self._handle is not None:
                try:
                    self._handle.write(
                        json.dumps(stored, ensure_ascii=False, sort_keys=True) + "\n"
                    )
                except OSError as exc:
                    persisted = False
                    self._record_write_error(exc)
                    self._volatile_appends.append(stored)
            elif self._path is not None:
                persisted = False
                self._volatile_appends.append(stored)
            self._preview.append(stored)
            self._count += 1
        return persisted

    def replace(self, index: int, row: dict[str, object]) -> bool:
        if not self.supports_updates:
            raise TypeError("this spool does not support row replacement")
        resolved = index if index >= 0 else self._count + index
        if resolved < 0 or resolved >= self._count:
            raise IndexError(index)
        stored = {key: _storage_values._serializable_value(value) for key, value in row.items()}
        record = {"__spool_update_index": resolved, "__spool_row": stored}
        persisted = True
        with self._lock:
            if self._handle is not None:
                try:
                    self._handle.write(
                        json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
                    )
                except OSError as exc:
                    persisted = False
                    self._record_write_error(exc)
                    self._volatile_updates[resolved] = stored
            elif self._path is not None:
                persisted = False
                self._volatile_updates[resolved] = stored
            preview_start = self._count - len(self._preview)
            if resolved >= preview_start:
                self._preview[resolved - preview_start] = stored
        return persisted

    def replace_with_admission(
        self,
        index: int,
        row: Mapping[str, object],
        *,
        lease: tuple[int, int] | None = None,
    ) -> RowWriteAdmission:
        """Atomically replace a row only while its spool is writable."""

        with self._lock:
            if not self.supports_updates:
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    "spool does not support row replacement",
                    row_index=index,
                    lease=(self._owner_epoch, index),
                )
            resolved = index if index >= 0 else self._count + index
            current_lease = (self._owner_epoch, resolved)
            if resolved < 0 or resolved >= self._count:
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    "spool row index is no longer available",
                    row_index=resolved,
                    lease=current_lease,
                )
            if lease is not None and lease != current_lease:
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    "spool row lease no longer owns this index",
                    row_index=resolved,
                    lease=current_lease,
                )
            if self._write_error is not None:
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    f"spool write failed: {self._write_error}",
                    row_index=resolved,
                    lease=current_lease,
                )
            if self._path is None or self._handle is None:
                return RowWriteAdmission(
                    ROW_WRITE_UNAVAILABLE,
                    "spool is unbound or closed",
                    row_index=resolved,
                    lease=current_lease,
                )

            stored = {
                key: _storage_values._serializable_value(value) for key, value in row.items()
            }
            if self._handle is not None:
                record = {"__spool_update_index": resolved, "__spool_row": stored}
                try:
                    self._handle.write(
                        json.dumps(record, ensure_ascii=False, sort_keys=True)
                        + "\n"
                    )
                except OSError as exc:
                    self._record_write_error(exc)
                    self._volatile_updates[resolved] = stored
                    preview_start = self._count - len(self._preview)
                    if resolved >= preview_start:
                        self._preview[resolved - preview_start] = stored
                    return RowWriteAdmission(
                        ROW_WRITE_UNAVAILABLE,
                        f"spool write failed: {exc}",
                        row_index=resolved,
                        lease=current_lease,
                    )
            preview_start = self._count - len(self._preview)
            if resolved >= preview_start:
                self._preview[resolved - preview_start] = stored
            return RowWriteAdmission(
                ROW_WRITE_ADMITTED,
                row_index=resolved,
                lease=current_lease,
            )

    def pop(self, index: int = -1) -> dict[str, object]:
        """Remove and return one row without rewriting the whole spool file.

        Mirrors ``replace()``: instead of materializing every row and
        rewriting the entire on-disk file (O(n) per call, O(n**2) for a run
        of pops), this appends a single tombstone record marking the row as
        deleted.  ``_materialized_rows()`` replays tombstones in order to
        reconstruct the current logical sequence.
        """

        if not self.supports_updates:
            raise TypeError("this spool does not support row removal")
        with self._lock:
            resolved = index if index >= 0 else self._count + index
            if resolved < 0 or resolved >= self._count:
                raise IndexError(index)
            removed = self[resolved]
            record = {"__spool_delete_index": resolved}
            if self._handle is not None:
                try:
                    self._handle.write(
                        json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
                    )
                except OSError as exc:
                    self._record_write_error(exc)
                    self._volatile_deletes.append(resolved)
            elif self._path is not None:
                self._volatile_deletes.append(resolved)
            # A pending (not-yet-persisted) replacement for this exact row is
            # moot now that the row itself is gone.
            self._volatile_updates.pop(resolved, None)
            preview_start = self._count - len(self._preview)
            if resolved >= preview_start:
                del self._preview[resolved - preview_start]
            self._count -= 1
            return removed

    def close(self) -> bool:
        success = True
        with self._lock:
            if self._handle is not None:
                handle = self._handle
                try:
                    handle.flush()
                    os.fsync(handle.fileno())
                except OSError as exc:
                    success = False
                    self._record_write_error(exc)
                else:
                    try:
                        handle.close()
                    except OSError as exc:
                        success = False
                        self._record_write_error(exc)
                    else:
                        self._handle = None
        return success

    def flush(self) -> bool:
        with self._lock:
            if self._handle is not None:
                try:
                    self._handle.flush()
                except OSError as exc:
                    self._record_write_error(exc)
                    return False
        return self._write_error is None

    def clear(self) -> None:
        with self._lock:
            self.close()
            self._owner_epoch += 1
            if self._path is not None:
                self._path.unlink(missing_ok=True)
            self._path = None
            self._preview.clear()
            self._count = 0
            self._volatile_appends.clear()
            self._volatile_updates.clear()
            self._volatile_deletes.clear()
            self._write_error = None

    def discard_spool(self) -> bool:
        """Best-effort removal after every final artifact is committed.

        Cleanup can fail because of filesystem permissions or transient I/O,
        but verified published data must not be downgraded into a retry that
        has already removed some sibling spools.  A failed path remains bound
        for later maintenance; callers never lose the full materialized rows.
        """
        with self._lock:
            if not self.close():
                return False
            path = self._path
            if path is not None:
                try:
                    path.unlink(missing_ok=True)
                except OSError:
                    return False
                try:
                    path.parent.rmdir()
                except OSError:
                    pass
            self._path = None
            self._owner_epoch += 1
            return True

    def __len__(self) -> int:
        return self._count

    @property
    def preview_rows(self) -> list[dict[str, object]]:
        return list(self._preview)

    def _plain_rows(self) -> Iterator[dict[str, object]]:
        path = self._path
        if path is None or not path.exists():
            yield from self._preview
            return
        with path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise OSError(
                        f"malformed spool JSONL row: path={path}, line={line_number}"
                    ) from exc
                if not isinstance(row, dict):
                    raise OSError(
                        f"spool JSONL row is not an object: path={path}, line={line_number}"
                    )
                if "__spool_update_index" not in row:
                    yield row
        yield from self._volatile_appends

    def _iter_replayed_rows(self) -> Iterator[dict[str, object]]:
        path = self._path
        if path is None or not path.exists():
            yield from self._preview
            return
        from .disk_index import DiskRowTable, disk_index

        with disk_index(path.parent) as connection:
            rows = DiskRowTable(connection, "spool_replay")
            with path.open("r", encoding="utf-8") as handle:
                for line_number, line in enumerate(handle, start=1):
                    if not line.strip():
                        continue
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError as exc:
                        raise OSError(
                            f"malformed spool JSONL row: path={path}, line={line_number}"
                        ) from exc
                    if not isinstance(record, dict):
                        raise OSError(
                            f"spool JSONL row is not an object: path={path}, line={line_number}"
                        )
                    if "__spool_update_index" in record:
                        try:
                            update_index = int(record["__spool_update_index"])
                            update_row = record["__spool_row"]
                            if not isinstance(update_row, dict):
                                raise TypeError("update row is not an object")
                            rows.replace(update_index, update_row)
                        except (IndexError, KeyError, TypeError, ValueError) as exc:
                            raise OSError(
                                f"invalid spool update record: path={path}, line={line_number}"
                            ) from exc
                    elif "__spool_delete_index" in record:
                        try:
                            delete_index = int(record["__spool_delete_index"])
                            rows.delete(delete_index)
                        except (IndexError, KeyError, TypeError, ValueError) as exc:
                            raise OSError(
                                f"invalid spool delete record: path={path}, line={line_number}"
                            ) from exc
                    else:
                        rows.append(record)
            for index, row in self._volatile_updates.items():
                if 0 <= index < len(rows):
                    rows.replace(index, row)
            rows.extend(self._volatile_appends)
            for index in self._volatile_deletes:
                if 0 <= index < len(rows):
                    rows.delete(index)
            yield from rows

    def _materialized_rows(self) -> list[dict[str, object]]:
        """Compatibility helper; save/recovery iteration uses the disk replay."""
        return list(self._iter_replayed_rows())

    def __iter__(self) -> Iterator[dict[str, object]]:
        # The disk read must happen under the same lock acquisition as the
        # path/count snapshot: releasing the lock in between lets a
        # concurrent append/bind/close/discard_spool truncate or rebind the
        # file out from under us, producing a spurious row-count mismatch or
        # FileNotFoundError.  Every other method on this class that touches
        # disk (open_existing, _repair_unterminated_tail, replace, ...) holds
        # ``self._lock`` for the full duration of its I/O; do the same here.
        with self._lock:
            self.flush()
            path = self._path
            preview = list(self._preview)
            count = self._count
            if path is None or not path.exists():
                yield from preview
                return
            yielded = 0
            source = self._iter_replayed_rows() if self.supports_updates else self._plain_rows()
            for row in source:
                yielded += 1
                yield row
            if yielded != count:
                raise OSError(f"spool row count mismatch: expected={count}, read={yielded}, path={path}")

    def __getitem__(self, index):
        if isinstance(index, slice):
            return list(self)[index]
        resolved = index if index >= 0 else self._count + index
        if resolved < 0 or resolved >= self._count:
            raise IndexError(index)
        preview_start = self._count - len(self._preview)
        if resolved >= preview_start:
            return list(self._preview)[resolved - preview_start]
        for current, row in enumerate(self):
            if current == resolved:
                return row
        raise IndexError(index)

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Sequence):
            return list(self) == list(other)
        return False

# Preserve existing class/pickle identity at the named compatibility boundary.
SpoolRows.__module__ = "Workstation.workstation_core.recording_session"
