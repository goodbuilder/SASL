"""Immutable Measurement start identity shared by every output producer.

This module owns names only. It never changes sample clocks or coordinates.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

KST = timezone(timedelta(hours=9), name="KST")


def measurement_path_sort_key(path: Path) -> tuple[str, int, int, str]:
    """Sort start time + collision number, rather than an older export's mtime."""
    match = re.search(r"(\d{8}_\d{6})(?:_(\d{9})\+0900)?(?:-(\d+))?", path.name)
    if match is None:
        return "", 0, 0, path.name
    return match[1], int(match[2] or 0), int(match[3] or 0), path.name


def measurement_start_stamp(utc_ns: int) -> str:
    seconds, nanos = divmod(int(utc_ns), 1_000_000_000)
    local = datetime.fromtimestamp(seconds, tz=KST)
    return f"{local:%Y%m%d_%H%M%S}_{nanos:09d}+0900"


@dataclass(frozen=True)
class MeasurementStorageContext:
    measurement_id: str
    started_utc_ns: int
    started_monotonic_ns: int
    file_stamp: str
    root: Path

    @property
    def workbook_path(self) -> Path:
        return self.root / f"{self.file_stamp}_measurement.xlsx"

    def stream_path(self, stem: str) -> Path:
        return self.root / "raw" / f"{self.file_stamp}_{stem}.jsonl"

    def metadata(self) -> dict[str, object]:
        return {
            "measurement_id": self.measurement_id,
            "measurement_started_utc_ns": self.started_utc_ns,
            "measurement_started_monotonic_ns": self.started_monotonic_ns,
            "measurement_file_stamp": self.file_stamp,
        }

    def allocate(self, category: str, label: str, suffix: str) -> Path:
        """Atomically reserve a filename; a late producer keeps this context."""
        def safe(value: str) -> str:
            return "".join(c if c.isalnum() or c in "-_" else "_" for c in value) or "artifact"

        if not suffix.startswith(".") or not suffix[1:].isalnum():
            raise ValueError("artifact suffix must be a short extension")
        directory = self.root / safe(category)
        directory.mkdir(parents=True, exist_ok=True)
        stem = f"{self.file_stamp}_{safe(label)}"
        sequence = 1
        while True:
            tail = "" if sequence == 1 else f"-{sequence:03d}"
            path = directory / f"{stem}{tail}{suffix.lower()}"
            try:
                fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
            except FileExistsError:
                sequence += 1
                continue
            os.close(fd)
            return path


def reserve_measurement_context(
    data_root: Path, measurement_id: str, started_utc_ns: int, started_monotonic_ns: int
) -> MeasurementStorageContext:
    """Reserve a whole run directory so collisions affect every file equally."""
    base = measurement_start_stamp(started_utc_ns)
    data_root = Path(data_root)
    data_root.mkdir(parents=True, exist_ok=True)
    sequence = 0
    while True:
        stamp = base if sequence == 0 else f"{base}-{sequence:02d}"
        root = data_root / stamp
        try:
            root.mkdir()
        except FileExistsError:
            sequence += 1
            continue
        return MeasurementStorageContext(
            str(measurement_id), int(started_utc_ns), int(started_monotonic_ns), stamp, root
        )
