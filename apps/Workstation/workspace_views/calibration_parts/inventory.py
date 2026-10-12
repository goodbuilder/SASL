"""File inventory and thumbnail reads for the calibration input presentation.

These functions perform no QWidget updates. The picker retains the existing
one-image-per-event-loop-turn scheduling and admits results using its generation.
No extra worker, thread, timer, or live backend lookup is created here.
"""
from __future__ import annotations
import json
from pathlib import Path
from PyQt5 import QtCore, QtGui
from Workstation.workstation_core.calibration_selection import CALIBRATION_IMAGE_SUFFIXES


def path_stamp(path: Path) -> tuple[int, int] | None:
    try:
        stat = path.stat()
    except OSError:
        return None
    return int(stat.st_mtime_ns), int(stat.st_size)

def directory_stamp(path: Path, prefix: str) -> tuple[object, ...] | None:
    try:
        stat = path.stat()
        names = tuple(
            sorted(
                entry.name
                for entry in path.iterdir()
                if entry.is_file() and entry.name.startswith(prefix) and entry.suffix.lower() == ".png"
            )
        )
    except OSError:
        return None
    # Directory mtime catches rewrites; names catch filesystems whose
    # directory mtime granularity is too coarse for rapid captures.
    return int(stat.st_mtime_ns), int(stat.st_size), names

def metadata_directory_stamp(path: Path) -> tuple[object, ...] | None:
    try:
        stat = path.stat()
        entries = tuple(
            sorted(
                (entry.name, int(entry.stat().st_mtime_ns), int(entry.stat().st_size))
                for entry in path.iterdir()
                if entry.is_file()
                and entry.name.startswith("capture_")
                and entry.suffix.lower() == ".json"
            )
        )
    except OSError:
        return None
    return int(stat.st_mtime_ns), int(stat.st_size), entries


def pair_capture_time_ns(root: Path, key: str) -> int:
    metadata_path = root / "capture_metadata" / f"capture_{key}.json"
    try:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        payload = {}
    if isinstance(payload, dict):
        for field in ("capture_recorded_unix_ns", "capture_requested_unix_ns"):
            try:
                value = int(payload.get(field))
            except (TypeError, ValueError):
                continue
            if value > 0:
                return value
    try:
        return int((root / "left" / f"left_{key}.png").stat().st_mtime_ns)
    except OSError:
        return 0


def image_candidates(directory: Path) -> list[Path]:
    return sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in CALIBRATION_IMAGE_SUFFIXES
    ) if directory.is_dir() else []


def paired_keys(root: Path) -> list[str]:
    left_dir, right_dir = root / "left", root / "right"
    left = {path.stem.removeprefix("left_") for path in left_dir.glob("left_*.png")} if left_dir.is_dir() else set()
    right = {path.stem.removeprefix("right_") for path in right_dir.glob("right_*.png")} if right_dir.is_dir() else set()
    return sorted(left & right)


def read_thumbnail(path: Path) -> QtGui.QImage:
    image = QtGui.QImage(str(path))
    return image.scaledToHeight(36, QtCore.Qt.SmoothTransformation) if not image.isNull() else image
