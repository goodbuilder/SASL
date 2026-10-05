"""CSV mirrors for JSONL ledgers owned by binary-evidence workers.

Call only after the owning worker durably commits its JSONL row. A CSV
failure never rolls back the verified binary/JSONL pair. The next call
rebuilds from JSONL, including rows lost during an earlier process crash.
"""

from __future__ import annotations

import csv
from contextlib import contextmanager
from itertools import zip_longest
import os
from pathlib import Path
import threading

from Workstation.workstation_core.measurement_data_store import CSV_FIELDNAMES, MeasurementDataStore

_ready: set[Path] = set()
_guard = threading.Lock()
_locks: dict[Path, threading.RLock] = {}
_failed: dict[Path, str] = {}


@contextmanager
def owned_ledger_lock(source: Path):
    with _guard:
        lock = _locks.setdefault(Path(source), threading.RLock())
    with lock:
        yield


def failed_mirrors() -> dict[Path, str]:
    with _guard:
        return dict(_failed)


def repair_owned_jsonl_mirror(source: Path) -> Path:
    """Verify first; replace only a mismatching CSV, keeping JSONL untouched."""
    source = Path(source)
    target = source.with_suffix(".csv")
    with owned_ledger_lock(source):
        equal = False
        if target.is_file() and not target.is_symlink():
            try:
                with source.open(encoding="utf-8") as raw, target.open(encoding="utf-8", newline="") as mirrored:
                    reader = csv.DictReader(mirrored)
                    equal = tuple(reader.fieldnames or ()) == CSV_FIELDNAMES and all(
                        line is not None and row is not None and line.endswith("\n") and line.rstrip("\n") == row.get("record_json")
                        for line, row in zip_longest(raw, reader)
                    )
            except (OSError, csv.Error, UnicodeError):
                equal = False
        if equal:
            with _guard:
                _failed.pop(source, None)
            return target
        _ready.discard(target)
        return mirror_owned_jsonl_record(source, b"")


def mirror_owned_jsonl_record(source: Path, payload: bytes) -> Path:
    source = Path(source)
    target = source.with_suffix(".csv")
    temporary = target.with_name(f".{target.name}.repair.tmp")
    with owned_ledger_lock(source):
        try:
            if target.is_symlink() or (target.exists() and not target.is_file()):
                raise OSError("CSV mirror is not a regular file")
            if target not in _ready or not target.exists():
                if target.exists() and target.stat().st_size:
                    with target.open(encoding="utf-8", newline="") as old:
                        if tuple(next(csv.reader(old), ())) != CSV_FIELDNAMES:
                            raise OSError("CSV header is invalid or foreign")
                with source.open("rb") as raw, temporary.open("w", encoding="utf-8", newline="") as output:
                    writer = csv.DictWriter(output, fieldnames=CSV_FIELDNAMES, lineterminator="\n")
                    writer.writeheader()
                    for line in raw:
                        if not line.endswith(b"\n"):
                            raise OSError("incomplete JSONL row")
                        writer.writerow(MeasurementDataStore._csv_row_from_payload(line))
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(temporary, target)
                _ready.add(target)
            else:
                with target.open("a", encoding="utf-8", newline="") as output:
                    csv.DictWriter(output, fieldnames=CSV_FIELDNAMES, lineterminator="\n").writerow(
                        MeasurementDataStore._csv_row_from_payload(payload)
                    )
                    output.flush()
                    os.fsync(output.fileno())
            with _guard:
                _failed.pop(source, None)
            return target
        except Exception as exc:
            _ready.discard(target)
            with _guard:
                _failed[source] = str(exc)
            raise
        finally:
            temporary.unlink(missing_ok=True)
