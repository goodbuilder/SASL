"""Small, filesystem-backed calibration-session catalog.

The calibration UI and the owner controller both treat a session root as the
unit of capture, computation, and result persistence.  Discovery is tolerant
of older layouts: any directory containing matching ``left_*.png`` and
``right_*.png`` pairs is a selectable offline photo set.
"""

from __future__ import annotations

import json
import stat as stat_module
import time
from dataclasses import dataclass
from pathlib import Path

from Workstation.workstation_core.calibration_selection import CALIBRATION_IMAGE_SUFFIXES
from Workstation.workstation_core.calibration_timing import (
    INVALID_METADATA,
    LEGACY_UNVERIFIED,
    classify_calibration_pair_usage,
    load_calibration_pair_usage,
)


SESSION_MANIFEST_NAME = "calibration_session.json"


@dataclass(frozen=True)
class CalibrationSession:
    root: Path
    session_id: str
    pair_count: int
    updated_ns: int
    manifest_path: Path

    @property
    def display_name(self) -> str:
        return f"{self.session_id} · {self.pair_count} pair"


@dataclass(frozen=True)
class CalibrationCaptureInventory:
    """Filesystem-backed counts for one calibration capture/photo-set root.

    Counts are evidence, not solver readiness.  ``stereo_candidate_count``
    follows the same persisted pair-usage decision as the calibration input
    path, before manual exclusions or solver pose filtering.
    """

    root: Path
    state: str
    left_image_count: int | None
    right_image_count: int | None
    stereo_candidate_count: int | None
    left_intrinsics_only_count: int | None
    right_intrinsics_only_count: int | None
    legacy_unverified_pair_count: int | None
    invalid_metadata_pair_count: int | None
    error: str = ""

    @property
    def evidence_warning_pair_count(self) -> int | None:
        if (
            self.legacy_unverified_pair_count is None
            or self.invalid_metadata_pair_count is None
        ):
            return None
        return self.legacy_unverified_pair_count + self.invalid_metadata_pair_count

    def to_mapping(self) -> dict[str, object]:
        return {
            "root": str(self.root),
            "state": self.state,
            "left_image_count": self.left_image_count,
            "right_image_count": self.right_image_count,
            "stereo_candidate_count": self.stereo_candidate_count,
            "left_intrinsics_only_count": self.left_intrinsics_only_count,
            "right_intrinsics_only_count": self.right_intrinsics_only_count,
            "legacy_unverified_pair_count": self.legacy_unverified_pair_count,
            "invalid_metadata_pair_count": self.invalid_metadata_pair_count,
            "evidence_warning_pair_count": self.evidence_warning_pair_count,
            "error": self.error,
        }


def _calibration_side_paths(root: Path, side: str) -> list[Path]:
    directory = root / side
    try:
        directory_stat = directory.stat()
    except FileNotFoundError:
        return []
    if not stat_module.S_ISDIR(directory_stat.st_mode):
        raise NotADirectoryError(f"{directory} is not a directory")
    paths: list[Path] = []
    for path in directory.iterdir():
        if path.suffix.lower() not in CALIBRATION_IMAGE_SUFFIXES:
            continue
        if stat_module.S_ISREG(path.stat().st_mode):
            paths.append(path)
    return sorted(paths)


def scan_calibration_capture_inventory(
    root: str | Path,
) -> CalibrationCaptureInventory:
    """Scan one root without turning missing/unreadable evidence into zero."""

    capture_root = Path(root).expanduser()
    try:
        try:
            root_stat = capture_root.stat()
        except FileNotFoundError:
            root_stat = None
        if root_stat is not None and not stat_module.S_ISDIR(root_stat.st_mode):
            raise NotADirectoryError(f"{capture_root} is not a directory")
        left_paths = _calibration_side_paths(capture_root, "left")
        right_paths = _calibration_side_paths(capture_root, "right")
        left_by_key = {
            path.stem.removeprefix("left_"): path for path in left_paths
        }
        right_by_key = {
            path.stem.removeprefix("right_"): path for path in right_paths
        }
        common_keys = sorted(set(left_by_key) & set(right_by_key))
        if common_keys:
            pairs = [
                (
                    key,
                    left_by_key[key],
                    right_by_key[key],
                    load_calibration_pair_usage(
                        capture_root / "capture_metadata" / f"capture_{key}.json"
                    ),
                )
                for key in common_keys
            ]
        elif left_paths and len(left_paths) == len(right_paths):
            legacy_usage = classify_calibration_pair_usage(None)
            pairs = [
                (f"ordered-{index:06d}", left, right, legacy_usage)
                for index, (left, right) in enumerate(
                    zip(left_paths, right_paths, strict=False)
                )
            ]
        else:
            pairs = []
    except OSError as exc:
        return CalibrationCaptureInventory(
            root=capture_root,
            state="UNAVAILABLE",
            left_image_count=None,
            right_image_count=None,
            stereo_candidate_count=None,
            left_intrinsics_only_count=None,
            right_intrinsics_only_count=None,
            legacy_unverified_pair_count=None,
            invalid_metadata_pair_count=None,
            error=str(exc),
        )

    eligible_left = {left for _key, left, _right, usage in pairs if usage.stereo_eligible}
    eligible_right = {right for _key, _left, right, usage in pairs if usage.stereo_eligible}
    return CalibrationCaptureInventory(
        root=capture_root,
        state="READY" if left_paths or right_paths else "EMPTY",
        left_image_count=len(left_paths),
        right_image_count=len(right_paths),
        stereo_candidate_count=sum(1 for *_paths, usage in pairs if usage.stereo_eligible),
        left_intrinsics_only_count=len(left_paths) - len(eligible_left),
        right_intrinsics_only_count=len(right_paths) - len(eligible_right),
        legacy_unverified_pair_count=sum(
            1 for *_paths, usage in pairs if usage.state == LEGACY_UNVERIFIED
        ),
        invalid_metadata_pair_count=sum(
            1 for *_paths, usage in pairs if usage.state == INVALID_METADATA
        ),
    )


def _pair_count(root: Path) -> int:
    count = scan_calibration_capture_inventory(root).stereo_candidate_count
    return 0 if count is None else count


def discover_calibration_sessions(base_root: str | Path) -> tuple[CalibrationSession, ...]:
    base = Path(base_root).expanduser()
    if not base.is_dir():
        return ()
    candidates: set[Path] = set()
    for left_dir in base.rglob("left"):
        if not left_dir.is_dir() or not (left_dir.parent / "right").is_dir():
            continue
        if not any(left_dir.glob("left_*.png")) or not any(
            (left_dir.parent / "right").glob("right_*.png")
        ):
            continue
        candidates.add(left_dir.parent)
    sessions: list[CalibrationSession] = []
    for root in candidates:
        try:
            updated_ns = max(
                path.stat().st_mtime_ns
                for path in (root / "left").glob("left_*.png")
            )
        except (OSError, ValueError):
            continue
        sessions.append(
            CalibrationSession(
                root=root,
                session_id=root.name,
                pair_count=_pair_count(root),
                updated_ns=int(updated_ns),
                manifest_path=root / SESSION_MANIFEST_NAME,
            )
        )
    return tuple(sorted(sessions, key=lambda item: (item.updated_ns, str(item.root)), reverse=True))


def write_calibration_session_manifest(
    root: str | Path,
    *,
    session_kind: str = "calibration",
    source: str = "",
) -> Path:
    session_root = Path(root)
    path = session_root / SESSION_MANIFEST_NAME
    previous: dict[str, object] = {}
    if path.exists():
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                previous = loaded
        except (OSError, ValueError, TypeError):
            previous = {}
    payload = {
        "schema": "workstation-calibration-session-v1",
        "session_id": session_root.name,
        "session_kind": session_kind,
        "created_unix_ns": int(previous.get("created_unix_ns") or time.time_ns()),
        "updated_unix_ns": time.time_ns(),
        "root": str(session_root),
        "last_source": str(source or previous.get("last_source") or ""),
        "pair_count": _pair_count(session_root),
        "source_roots": {
            "HR": str(session_root / "calib_capture" / "HR"),
            "OZ": str(session_root / "calib_capture" / "OZ"),
            "AutoCapture": str(session_root / "calib_capture" / "AutoCapture"),
        },
    }
    session_root.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return path
